#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include<linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>
#include <linux/i2c/msp430.h>

#define WDT_CTRL_REG          0x00
#define GINO_TASK_REG         0x01
#define DVI2LVDS_FLAGS_REG    0x02
#define DATA_REG              0x03
#define INDEX_REG             0x04
#define WDT_DELAY_REG         0x05
#define WDT_TIMEOUT_REG       0x06
#define WDT_TIME1_REG         0x07
#define WDT_TIME2_REG         0x08
#define WDT_CONFIG_REG        0x09
#define ADC_READING_0_REG     0x0A
#define ADC_READING_1_REG     0x0B
#define BUILDREV_REG          0x0C
#define FW_ID_REV_REG         0x0E

#define SBLOCK_CMD			  0x55AA
#define HALT_CMD			  0x6101
#define REBOOT_CMD			  0x6505

struct msp430 {
	struct i2c_client  *client;
	void 		   	   (*task_pre_halt_signal) (void);
	void               (*task_post_halt_signal) (void);
};

static struct msp430 *msp430;

static int PowerState;
static int StateValidate;


static struct msp430_reg_rw reg_halt[] = {
	{
		.op   = WVE_OP,
		.reg  = {
					.addr = 0x0001,
					.data = 0x55AA,
				},
	},
	{
		.op   = WVE_OP,
		.reg  = { 
					.addr = 0x0001,
					.data = HALT_CMD,
				},
	},
	{
		.op   = STOP_OP,
		.reg  = {
					.addr = 0,
					.data = 0,
			  	},
	},
};

static struct msp430_reg_rw reg_reboot[] = {
	{
		.op   = WVE_OP,
		.reg  = {
					.addr = 0x0001,
					.data = 0x55AA,
				},
	},
	{
		.op   = WVE_OP,
		.reg  = {
					.addr = 0x0001,
					.data = REBOOT_CMD,
				},
	},
	{
		.op   = STOP_OP,
		.reg  = {
					.addr = 0,
					.data = 0,
			  	},
	},
};

/************** BASILAR FUNCTIONS **************/

static inline u16 msp430_read_data (struct i2c_client *client, u16 addr) {
	s32 data;
	u16 val;
	data = i2c_smbus_read_word_data(client, (u8)addr);
	if (data < 0) {
		dev_err(&client->dev, "i2c io (read) error: %d\n", data);
		return data;
	}
	val = (u16)data;
	dev_dbg(&client->dev, "data: 0x%x, val: 0x%x\n", data, val);
	return val;
}


static inline u16 msp430_write_data (struct i2c_client *client, u16 addr, u16 data) {
	return i2c_smbus_write_word_data(client, (u8)addr, data);
}


static inline u16 msp430_read_vector_element (struct i2c_client *client, u16 addr) {
	int retval = msp430_write_data (client, INDEX_REG, addr);
	if (retval < 0) {
		retval = msp430_read_data (client, DATA_REG);
	}
	return (u16)retval;
}


static inline u16 msp430_write_vector_element (struct i2c_client *client, u16 addr, u16 data) {
	int retval = msp430_write_data (client, DATA_REG, addr);
	if (!(retval < 0)) {
		retval = msp430_write_data (client, INDEX_REG, data);
	}
	return (u16)retval;
}


static int msp430_mem_single_op (struct i2c_client *client, struct msp430_reg_rw *reg_rw) {
	int retval = 0;
	
	u16 val;
	switch (reg_rw->op) {
		case R_OP:
			val = msp430_read_data (client, reg_rw->reg.addr);
			if (val >= 0) {
				dev_dbg(&client->dev, "read data done: 0x%x\n", val);
				reg_rw->reg.data = val;	
				retval = 1;
			} else {
				dev_err(&client->dev, "read data failed: 0x%x\n", val);
				retval = -1;
			}
			break;
		case W_OP:
			val = msp430_write_data (client, reg_rw->reg.addr, reg_rw->reg.data);
			if (val >= 0) {
				dev_dbg(&client->dev, "write data done: addr 0x%x  data 0x%x\n", reg_rw->reg.addr, reg_rw->reg.data);
				retval = 0;
			} else {
				dev_err(&client->dev, "write data failed: addr 0x%x  data 0x%x\n", reg_rw->reg.addr, reg_rw->reg.data);
				retval = -1;
			}
			break;
		case RVE_OP:
			val = msp430_read_vector_element (client, reg_rw->reg.addr);
			if (val >= 0) {
				dev_dbg(&client->dev, "read vector element done: 0x%x", val);
				reg_rw->reg.data = val;
				retval = 1;
			} else {
				dev_err(&client->dev, "read vector's element failed: 0x%x\n", val);
				retval = -1;
			}
			break;
		case WVE_OP:
			val = msp430_write_vector_element (client, reg_rw->reg.addr, reg_rw->reg.data);
			if (val >= 0) {
				dev_dbg(&client->dev, "write vector's element done: addr 0x%x  data 0x%x\n", reg_rw->reg.addr, reg_rw->reg.data);
				retval = 1;		
			} else {
				dev_err(&client->dev, "write vector's element failed: addr 0x%x  data 0x%x\n", reg_rw->reg.addr, reg_rw->reg.data);
				retval = -1;
			}
			break;
		default:
			dev_dbg(&client->dev, "invalid operation!\n");
	 		retval = -1;
	}
	return retval;
}


static int msp430_mem_op (struct i2c_client *client, struct msp430_reg_rw regs[], struct data_list *data_l) {
	struct msp430_reg_rw *next = regs;
	struct data_list *tmp;
	int retval = 0;
	int val;
	for (; next->op != STOP_OP ; next++) {
		val = msp430_mem_single_op (client, next);
		if (val < 0)
			return val;
		if (val > 0) {
			if (data_l != NULL) {
				tmp = kzalloc (sizeof (struct data_list), GFP_KERNEL);
				tmp->data = next->reg.data;
				list_add (&(tmp->list), &(data_l->list));
			}
		}
		retval += val;
	}
	return retval;
}


static int get_reg_rw (struct msp430_reg_rw *reg_rw, struct msp430_reg reg, int op) {
	reg_rw->op = op;
	reg_rw->reg.data = reg.data;
	reg_rw->reg.addr = reg.addr;
	return 0;
}

/************** ADVANCED FUNCTIONS **************/

static int msp430_SystemHalt (struct i2c_client *client) {
	int retval;
	retval = msp430_mem_op (client, reg_halt, NULL);
	if (!(retval < 0)) {
		printk (KERN_INFO "Seco halt performed!\n");
	} else {
		printk (KERN_ERR "Seco halt not performed!\n");
	}
	return retval;
}


static int msp430_SystemReboot (struct i2c_client *client) {
	int retval;
	retval = msp430_mem_op (client, reg_reboot, NULL);
	if (!(retval < 0)) {
		printk (KERN_INFO "Seco reboot performed!\n");
	} else {
		printk (KERN_ERR "Seco reboot not performed!\n");
	}
	return retval;
}

/************** IOCTL **************/

static long msp430_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	struct i2c_client *client = file->private_data;
	int err = 0;
	int retval = 0;
	dev_dbg(&client->dev, "ioctl, cmd=0x%02x, arg=0x%02lx\n", cmd, arg);
	switch (cmd) {
		case MSP430_IOCTL_REG_READ: {
			struct msp430_reg reg;
			struct msp430_reg_rw reg_rw;
			if (copy_from_user (&reg, (const void __user *)arg, sizeof (reg))) {
				return -EFAULT;
			}
			get_reg_rw (&reg_rw, reg, R_OP);
			err = msp430_mem_single_op (client, &reg_rw);
			reg.data = reg_rw.reg.data;
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &reg, sizeof (reg))) {
				retval = -EFAULT;
			}
			break;
		}
		case MSP430_IOCTL_REG_WRITE: {
			struct msp430_reg reg;
			struct msp430_reg_rw reg_rw;
			if (copy_from_user (&reg, (const void __user *)arg, sizeof (reg))) {
				return -EFAULT;
			}
			get_reg_rw (&reg_rw, reg, W_OP);
			err = msp430_mem_single_op (client, &reg_rw);
			reg.data = reg_rw.reg.data;
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &reg, sizeof (reg))) {
				retval = -EFAULT;
			}
			break;
		}
		default:
			break;
	}
	return retval;
}


static int msp430_open(struct inode *inode, struct file *file) {
	file->private_data = msp430->client;
	return 0;
}


int msp430_release(struct inode *inode, struct file *file) {
	file->private_data = NULL;
	return 0;
}


static const struct file_operations msp430_fileops = {
	.owner = THIS_MODULE,
	.open = msp430_open,
	.unlocked_ioctl = msp430_ioctl,
	.release = msp430_release,
};


static struct miscdevice msp430_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msp430",
	.fops = &msp430_fileops,
};

/************************************/

static int __devinit msp430_probe(struct i2c_client *client, const struct i2c_device_id *id) {

	int retval = 0;
	struct msp430_halt_platform_data *pdata = client->dev.platform_data;
	int err;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA | I2C_FUNC_I2C))
		return -EIO;


	retval = misc_register(&msp430_device);

	msp430 = kzalloc (sizeof (struct msp430), GFP_KERNEL);
	
	if (!msp430) {
		err = -ENOMEM;
		goto err_free_mem;
	}
		
	msp430->client = client;

	SetPowerState (0x0, 0);

	printk (KERN_INFO "msp430 probe done\n");

	return 0;

 err_free_mem:
	kfree (msp430);
	return err;

}


static int __devexit msp430_remove(struct i2c_client *client) {
	struct msp430_halt_platform_data *data;
	data = i2c_get_clientdata(client);
	misc_deregister (&msp430_device);
	kfree (data);
	return 0;
}


void SetPowerState (int state, int validate) {
	PowerState = state;
	StateValidate = validate;
}


static const struct i2c_device_id msp430_idtable[] = {
	{ "msp430", 0 },
	{ }
};


MODULE_DEVICE_TABLE(i2c, msp430_idtable);


void device_shutdown_msp(struct i2c_client *client) {
	int retval;
	if (StateValidate) {
		switch (PowerState) {
			case LINUX_REBOOT_CMD_RESTART:
				retval = msp430_SystemReboot (client);
				break;
			case LINUX_REBOOT_CMD_HALT:
				retval = msp430_SystemHalt (client);
				break;
			case LINUX_REBOOT_CMD_CAD_ON:
				break;
			case LINUX_REBOOT_CMD_CAD_OFF:
				break;
			case LINUX_REBOOT_CMD_POWER_OFF:
				break;
			case LINUX_REBOOT_CMD_RESTART2:
				break;
			case LINUX_REBOOT_CMD_SW_SUSPEND:
				break;
			case LINUX_REBOOT_CMD_KEXEC:
				break;
			default:
				break;
		}
	}
}


static struct i2c_driver msp430_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "msp430"
	},
	.id_table	= msp430_idtable,
	.probe		= msp430_probe,
	.remove		= __devexit_p(msp430_remove),
	.shutdown   = device_shutdown_msp,
	
};


static int __init msp430_init(void) {
	return i2c_add_driver(&msp430_driver);
}


static void __exit msp430_exit(void) {
	i2c_del_driver(&msp430_driver);
}


module_init(msp430_init);
module_exit(msp430_exit);


MODULE_AUTHOR("DC SECO");
MODULE_DESCRIPTION("SECO system halt with MSP430 microcontroller");
MODULE_LICENSE("GPL");
