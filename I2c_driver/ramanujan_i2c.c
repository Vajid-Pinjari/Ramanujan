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

#define I2C_CONTROL_REG 0x00
#define I2C_STATUS_REG 0x04
#define I2C_DLEN_REG 0x08
#define I2C_SLAVE_ADDR_REG 0x0C
#define I2C_DATA_FIFO_REG 0x10
#define I2C_CLK_DEV_REG 0x14
#define I2C_DATA_DELAY_REG 0x18
#define I2C_CLK_STRETCH 0x1C

struct i2c_device{
	void __iomem *base;
	int irq;
	struct i2c_adapter adap;
	struct completion xfer_done;
};


/*Driver Metadata*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramanujan SOC :Vajid-Mirafra");
MODULE_DESCRIPTION("I2C Driver for RAMANUJAN");
