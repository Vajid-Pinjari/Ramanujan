/* Writing a I2C driver for Ramanujan SOC*/
#include<linux/uacess.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>









/*Driver Metadata*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramanujan SOC :Vajid-Mirafra");
MODULE_DESCRIPTION("I2C Driver for RAMANUJAN");
