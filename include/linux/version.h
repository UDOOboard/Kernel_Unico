#define LINUX_VERSION_CODE 196643
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#define SECO_VERSION ""
#define PRINT_VERSION printk (KERN_INFO "Kernel ver: %s\n", SECO_VERSION)
