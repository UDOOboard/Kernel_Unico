#ifndef __LINUX_I2C_TSC2004_H
#define __LINUX_I2C_TSC2004_H

/* linux/i2c/tsc2004.h */

struct tsc2004_platform_data {
	u16	model;				/* 2004. */
	u16	x_plate_ohms;
	u16	max_rt; /* max. resistance above which samples are ignored */
	unsigned long poll_delay; /* delay (in ms) after pen-down event
				     before polling starts */
	unsigned long poll_period; /* time (in ms) between samples */
	int	fuzzx; /* fuzz factor for X, Y and pressure axes */
	int	fuzzy;
	int	fuzzz;

	int	(*get_pendown_state)(void);
	void	(*clear_penirq)(void);		/* If needed, clear 2nd level
						   					interrupt source */
	int	(*init_platform_hw)(void);
	void	(*exit_platform_hw)(void);
};

#endif
