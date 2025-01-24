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

#define THIS_MODULE "mfr_i2c"
#define I2C_CONTROL_REG    0x00
#define I2C_STATUS_REG     0x04
#define I2C_DLEN_REG       0x08
#define I2C_SLAVE_ADDR_REG 0x0C
#define I2C_DATA_FIFO_REG  0x10
#define I2C_CLK_DEV_REG    0x14
#define I2C_DATA_DELAY_REG 0x18
#define I2C_CLK_STRETCH    0x1C

//I2C control Register bits
#define I2C_C_READ  BIT(0) //Read Bit
#define I2C_C_CLEAR BIT(4) //BIT 4 & 5.
#define I2C_C_ST    BIT(7) //Start Xfer.
#define I2C_C_INTD  BIT(8) //Int on Done.
#define I2C_C_INTT  BIT(9) //Int on Xfer.
#define I2C_C_INTR  BIT(10) //Int on RX.
#define I2C_C_EN    BIT(15)  //Enable I2C.

//I2C Status Register
#define I2C_S_TA	BIT(0) //Xfer Active.
#define I2C_S_DONE	BIT(1) //Xfer Done.
#define I2C_S_TXW	BIT(2) //FIFO needs writing.
#define I2C_S_RXR	BIT(3) //FIFO needs Reading.
#define I2C_S_TXD	BIT(4) //Accept Data.
#define I2C_S_RXD	BIT(5) //Contains Data.
#define I2C_S_TXE	BIT(6) //Empty.
#define I2C_S_RXF	BIT(7) //Full.
#define I2C_S_ERR	BIT(8) //ACK err.
#define I2C_S_CLKT	BIT(9) //Clock Stretch Timeout.

//I2C DEL(Delay) Register
#define I2C_FEDL_SHIFT	16 //Falling Edge Delay.
#define I2C_REDL_SHIFT	0  //Rising Edge Delay.

//I2C CDIV Register
/* Unreferenced */
#define I2C_CDIV_MIN	0x0002
#define I2C_CDIV_MAX	0xFFFE




struct mfr_i2c_device{
	struct device *dev;
	void __iomem *regs;
	int irq;
	struct i2c_adapter adapter;
	struct completion completion;
	struct i2c_msg *curr_msg;
	struct clk *bus_clk;
	int num_msgs;
	u32 msg_err;
	u8 *msg_buf;
	size_t msg_buf_remaining;
};




static int mfr_i2c_probe(struct platform_device *pdev)
{
    struct bcm2835_i2c_dev *i2c_dev;
    int ret;
    struct i2c_adapter *adap;
    struct clk *mclk;
    u32 bus_clk_rate;

    // Step 1: Allocate and initialize memory for the device structure
	
    // Step 2: Store the device structure for future reference
	
    // Step 3: Initialize device-specific completion mechanism

    // Step 4: Map hardware registers for I2C
	
    // Step 5: Get the main clock for the device

    // Step 6: Register and configure the bus clock divider
	
    // Step 7: Read the clock frequency from the device tree
	
    //         (default to standard I2C frequency if unavailable)

    // Step 8: Set the exclusive clock rate for the I2C bus
	
    // Step 9: Prepare and enable the clock for use

    // Step 10: Retrieve the interrupt line for the device
	
    // Step 11: Request an IRQ and set the ISR for handling interrupts

    // Step 12: Initialize the I2C adapter structure
	
    // Step 13: Set adapter metadata (name, owner, algorithm, parent, etc.)
	
    // Step 14: Register the adapter with the I2C subsystem

    // Step 15: Configure hardware registers (disable clock stretching timeout)
	
    // Step 16: Write default configurations to hardware registers

    // Step 17: Add the adapter to the I2C framework
	
    // Error Handling: Free IRQ, disable clock, and cleanup on failure

    return 0;
}

static int mfr_i2c_remove(struct platform_device *pdev)
{
	
}


static const struct of_device_id mfr_i2c_of_match[]={
	{ 
		.name = "mfr_i2c",
		.type = "i2c",
		.compatible = "mfr_i2c0",
	},
	{ .compatible = "mfr_i2c1" , .data = &mfr_i2c_quirks },
	{},
};
MODULE_DEVICE_TABLE(of, mfr_i2c_of_match);

static struct platform_driver i2c_platform_driver = {
.probe = mfr_i2c_probe,
.remove = mfr_i2c_remove,
.driver = {
	.name = "mfr_i2c",
	.of_match_table = mfr_i2c_of_match,
},
};

module_platform_driver(i2c_platform_driver);


/*Driver Metadata*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramanujan SOC :Vajid-Mirafra");
MODULE_DESCRIPTION("I2C Driver for RAMANUJAN");
