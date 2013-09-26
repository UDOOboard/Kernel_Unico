#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/delay.h>

#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>

#include <mach/iomux-mx6q.h>
#include <mach/iomux-mx6dl.h>


/************* UDOO USB-TO-SERIAL INTERFACES ************/
#define MX6QSDL_SECO_BOSSAC_CLK           IMX_GPIO_NR(6, 3)       // CLK  LINE CONTROLLED BY BOSSAC on Arduino IDE programming interface [ RTS_ARD ]
#define MX6QSDL_SECO_BOSSAC_DAT           IMX_GPIO_NR(5, 18)      // DATA LINE CONTROLLED BY BOSSAC on Arduino IDE programming interface [ DTR_ARD ]
#define MX6QSDL_SECO_ARD_ERASE            IMX_GPIO_NR(4, 21)      // ARDUINO ERASE LINE         [ C_ERASE ]
#define MX6QSDL_SECO_ARD_RESET            IMX_GPIO_NR(1, 0)       // ARDUINO MASTER_RESET LINE  [ IMX6_PIC_RST ]

#define MX6QSDL_SECO_KEYCOL0__GPIO_4_6    IMX_GPIO_NR(4, 6)
#define MX6QSDL_SECO_KEYROW0__GPIO_4_7    IMX_GPIO_NR(4, 7)


#define AUTH_TOKEN 0x5A5A
// #define MAX_MSEC_SINCE_LAST_IRQ 1000*1000*1000
#define MAX_MSEC_SINCE_LAST_IRQ 400 //

static struct workqueue_struct *erase_reset_wq;

typedef struct {
    struct work_struct erase_reset_work;
    int    step;
    int    cmdcode;
    unsigned long    last_int_time_in_ns;
    unsigned long    last_int_time_in_sec;
} erase_reset_work_t;

erase_reset_work_t *work;

static iomux_v3_cfg_t mx6q_uart4_pads[] = {
        MX6Q_PAD_KEY_ROW0__UART4_RXD,
        MX6Q_PAD_KEY_COL0__UART4_TXD,
};

static iomux_v3_cfg_t mx6dl_uart4_pads[] = {
        MX6DL_PAD_KEY_ROW0__UART4_RXD,
        MX6DL_PAD_KEY_COL0__UART4_TXD,
};

static iomux_v3_cfg_t mx6qd_disable_uart4_pads[] = {
        MX6Q_PAD_KEY_ROW0__GPIO_4_7,
        MX6Q_PAD_KEY_COL0__GPIO_4_6,
};

static iomux_v3_cfg_t mx6dl_disable_uart4_pads[] = {
        MX6DL_PAD_KEY_ROW0__GPIO_4_7,
        MX6DL_PAD_KEY_COL0__GPIO_4_6,
};


static void erase_reset_wq_function( struct work_struct *work)
{
    int ret;

    if (cpu_is_mx6q()){
        mxc_iomux_v3_setup_multiple_pads(mx6qd_disable_uart4_pads,
            ARRAY_SIZE(mx6qd_disable_uart4_pads));
            printk("Disable serial on UDOO Quad. \n");
    }
    else if (cpu_is_mx6dl()) {
        mxc_iomux_v3_setup_multiple_pads(mx6dl_disable_uart4_pads,
            ARRAY_SIZE(mx6dl_disable_uart4_pads));
            printk("Disable serial on UDOO Dual-lite. \n");
    }

    ret = gpio_request(MX6QSDL_SECO_KEYCOL0__GPIO_4_6, "SERIAL INPUT");
    if (ret) {
            printk(KERN_ERR "request GPIO FOR MX6QSDL_SECO_KEYCOL0__GPIO_4_6 IRQ\n");
    } else {
            gpio_direction_input(MX6QSDL_SECO_KEYCOL0__GPIO_4_6);
    }

    ret = gpio_request(MX6QSDL_SECO_KEYROW0__GPIO_4_7, "SERIAL INPUT");
    if (ret) {
            printk(KERN_ERR "request GPIO FOR MX6QSDL_SECO_KEYROW0__GPIO_4_7 IRQ\n");
    } else {
            gpio_direction_input(MX6QSDL_SECO_KEYROW0__GPIO_4_7);
    }

    gpio_direction_input(MX6QSDL_SECO_ARD_ERASE);
    gpio_set_value(MX6QSDL_SECO_ARD_RESET, 1);
    msleep(1);

    gpio_direction_output(MX6QSDL_SECO_ARD_ERASE, 1);
    msleep(300);
    gpio_direction_input(MX6QSDL_SECO_ARD_ERASE);

    msleep(10);
    gpio_set_value(MX6QSDL_SECO_ARD_RESET, 0);

    msleep(80);
    gpio_set_value(MX6QSDL_SECO_ARD_RESET, 1);
    //  kfree( (void *)work );
    printk("UDOO ERASE and RESET on Sam3x EXECUTED. \n");

//    msleep(30000);

//    if (cpu_is_mx6q()){
//        mxc_iomux_v3_setup_multiple_pads(mx6q_uart4_pads,
//            ARRAY_SIZE(mx6q_uart4_pads));
//            printk("Re-enable serial on UDOO quad. \n");
//    }
//    else if (cpu_is_mx6dl()) {
//        mxc_iomux_v3_setup_multiple_pads(mx6dl_uart4_pads,
//            ARRAY_SIZE(mx6dl_uart4_pads));
//            printk("Re-enable serial on UDOO dual-lite. \n");
//    }

    return;
}


static irqreturn_t udoo_bossac_req(int irq, void *dev_id)
{
    int     retval, auth_bit, expected_bit;
    int     cmdcode = 0x0;
    int     msec_since_last_irq;
    u64        nowsec;
    unsigned long  rem_nsec;

    auth_bit = gpio_get_value(MX6QSDL_SECO_BOSSAC_DAT);

    erase_reset_work_t *erase_reset_work = (erase_reset_work_t *)work;

    nowsec = local_clock();
    rem_nsec = do_div(nowsec, 1000000000) ;
    msec_since_last_irq = (((unsigned long)nowsec * 1000) + rem_nsec/1000000 ) - (((unsigned long)erase_reset_work->last_int_time_in_sec * 1000) + erase_reset_work->last_int_time_in_ns/1000000);

    if (msec_since_last_irq > MAX_MSEC_SINCE_LAST_IRQ) {
        erase_reset_work->step = 0;
// printk("Warn! UDOO KEY RESET AUTHENTICATION RESETTING TIMEOUT !\n");
    }

// printk("STEP %d -> 0x%d \n", erase_reset_work->step, auth_bit);
    erase_reset_work->last_int_time_in_ns = rem_nsec;
    erase_reset_work->last_int_time_in_sec = nowsec;

    if ( erase_reset_work->step < 16 ) {  // Authenticating received token bit.
        expected_bit = (( AUTH_TOKEN >> erase_reset_work->step ) & 0x01 );
        if ( auth_bit == expected_bit ) {
            erase_reset_work->step = erase_reset_work->step + 1;
        } else {
            erase_reset_work->step = 0;
        }
    } else { // Passed all authentication step. Receiving command code.
        erase_reset_work->cmdcode = erase_reset_work->cmdcode | (auth_bit << (erase_reset_work->step - 16));
        erase_reset_work->step = erase_reset_work->step + 1;
    }

    if ( erase_reset_work->step == 21 ) {  // Passed authentication and code acquiring step.

//printk("RECEIVED CODE = 0x%04x \n", erase_reset_work->cmdcode);
        if (erase_reset_work->cmdcode == 0xF) {
            retval = queue_work( erase_reset_wq, (struct work_struct *)work );
            // To do: check retval error code.
        } else {
//            msleep(30000);
            if (cpu_is_mx6q()){
                mxc_iomux_v3_setup_multiple_pads(mx6q_uart4_pads,
                ARRAY_SIZE(mx6q_uart4_pads));
                printk("Re-enable serial on UDOO quad. \n");
            }
            else if (cpu_is_mx6dl()) {
                mxc_iomux_v3_setup_multiple_pads(mx6dl_uart4_pads,
                ARRAY_SIZE(mx6dl_uart4_pads));
                printk("Re-enable serial on UDOO dual-lite. \n");
            }
        }
        erase_reset_work->step = 0;
        erase_reset_work->cmdcode = 0;
    }

    return IRQ_HANDLED;
}

static int gpio_setup(void)
{
    int ret;
    ret = gpio_request(MX6QSDL_SECO_BOSSAC_CLK, "BOSSA_CLK");
    if (ret) {
            printk(KERN_ERR "request BOSSA_CLK IRQ\n");
            return -1;
    } else {
            gpio_direction_input(MX6QSDL_SECO_BOSSAC_CLK);
    }

    ret = gpio_request(MX6QSDL_SECO_BOSSAC_DAT, "BOSSA_DAT");
    if (ret) {
            printk(KERN_ERR "request BOSSA_DAT IRQ\n");
            return -1;
    } else {
            gpio_direction_input(MX6QSDL_SECO_BOSSAC_DAT);
    }

    ret = gpio_request(MX6QSDL_SECO_ARD_ERASE, "BOSSAC");
    if (ret) {
            printk(KERN_ERR "request GPIO FOR ARDUINO ERASE\n");
            return -1;
    } else {
            gpio_direction_input(MX6QSDL_SECO_ARD_ERASE);
    }

    ret = gpio_request(MX6QSDL_SECO_ARD_RESET, "BOSSAC");
    if (ret) {
            printk(KERN_ERR "request GPIO FOR ARDUINO RESET\n");
            return -1;
    } else {
            gpio_direction_output(MX6QSDL_SECO_ARD_RESET, 1);
    }

    return 0;
}


static int udoo_ard_init(void)
{
    int retval;
    struct platform_device *bdev;
    bdev = kzalloc(sizeof(*bdev), GFP_KERNEL);

    gpio_setup();

    printk("registering IRQ %d for BOSSAC Arduino erase/reset operation\n", gpio_to_irq(MX6QSDL_SECO_BOSSAC_CLK));
    retval = request_irq(gpio_to_irq(MX6QSDL_SECO_BOSSAC_CLK), udoo_bossac_req, IRQF_TRIGGER_FALLING, "UDOO", bdev);

    erase_reset_wq = create_workqueue("erase_reset_queue");
    if (erase_reset_wq) {

        /* Queue some work (item 1) */
        work = (erase_reset_work_t *)kmalloc(sizeof(erase_reset_work_t), GFP_KERNEL);
        if (work) {
                 INIT_WORK( (struct work_struct *)work, erase_reset_wq_function );
                work->step = 1;
                work->cmdcode = 0;
                work->last_int_time_in_ns = 0;
                work->last_int_time_in_sec = 0;
            //  retval = queue_work( erase_reset_wq, (struct work_struct *)work );
        }
    }
    return  0;
}

static void udoo_ard_exit(void)
{
    printk("Unloading UDOO ard driver.\n");
    free_irq(gpio_to_irq(MX6QSDL_SECO_BOSSAC_CLK), NULL);

    gpio_free(MX6QSDL_SECO_ARD_RESET);
    gpio_free(MX6QSDL_SECO_ARD_ERASE);
    gpio_free(MX6QSDL_SECO_BOSSAC_CLK);
    gpio_free(MX6QSDL_SECO_BOSSAC_DAT);
    return;
}

module_init(udoo_ard_init);
module_exit(udoo_ard_exit);
