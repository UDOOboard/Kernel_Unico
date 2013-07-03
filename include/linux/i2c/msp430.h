#ifndef __LINUX_I2C_MSP430_H
#define __LINUX_I2C_MSP430_H

#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>

/* linux/i2c/msp430.h */

#define MSP430_IOCTL_REG_READ	 	_IOR('o', 1, struct msp430_reg)
#define MSP430_IOCTL_REG_WRITE		_IOWR('o', 2, struct msp430_reg)


struct msp430_platform_data {
	void (*task_pre_halt_signal) (void);
	void (*task_post_halt_signal) (void);
};

struct data_list {
	u16 data;
	struct list_head list;
};

enum {
	STOP_OP = (unsigned short int)0, 	// stop all operations
	R_OP    = (unsigned short int)1,	// read register operation
	W_OP    = (unsigned short int)2,	// write register operation
	RVE_OP  = (unsigned short int)3,	// read vector's element operation
	WVE_OP  = (unsigned short int)4,	// write vector's element operation 
};

struct msp430_reg {
	u16 addr;
	u16 data;
};	

struct msp430_reg_rw {
	unsigned short int op;
	struct msp430_reg reg;
};

#endif
