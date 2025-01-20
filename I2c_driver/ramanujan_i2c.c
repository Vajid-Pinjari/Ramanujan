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
/* Unreferenced 
#define BCM2835_I2C_CDIV_MIN	0x0002
#define BCM2835_I2C_CDIV_MAX	0xFFFE
*/



struct mfr_i2c_device{
	struct device *dev;
	void __iomem *regs;
	int irq;
	struct i2c_adapter adap;
	struct completion completion;
	struct i2c_msg *curr_msg;
	struct clk *bus_clk;
};

static struct clk *bcm2835_i2c_register_div(struct device *dev,
					struct clk *mclk,
					struct mfr_i2c_dev *i2c_dev)
{
	static struct clk *my_new_soc_i2c_register_div(struct device *dev,
                                               struct clk *mclk,
                                               struct my_new_soc_i2c_dev *i2c_dev)
{
    struct clk_init_data init;
    struct clk_mfr_i2c *priv;
    char name[32];
    const char *mclk_name;

    // Generating a name for the new clock
    snprintf(name, sizeof(name), "%s_div", dev_name(dev));

    // Fetch the parent clock name
    mclk_name = __clk_get_name(mclk);

    // Initialize the clock setup data
    init.ops = &clk_mfr_i2c_ops;  // Use the new SoC's clock ops
    init.name = name;
    init.parent_names = (const char* []) { mclk_name };
    init.num_parents = 1;
    init.flags = 0;

    // Allocate memory for the clock structure (specific to your new SoC)
    priv = devm_kzalloc(dev, sizeof(struct clk_my_new_soc_i2c), GFP_KERNEL);
    if (priv == NULL)
        return ERR_PTR(-ENOMEM);

    // Setup the hardware clock structure
    priv->hw.init = &init;
    priv->i2c_dev = i2c_dev;

    // Register the clock with the device
    clk_hw_register_clkdev(&priv->hw, "div", dev_name(dev));

    // Register the clock with the kernel (adjusted for your SoC)
    return devm_clk_register(dev, &priv->hw);
}

}

static int mfr_i2c_probe(struct platform_device *pdev)
{
	struct mfr_i2c_device *i2c_dev;
	int ret;
	struct i2c_adapter *adap;
	struct clk *mclk;
	u32 bus_clk_rate;

	i2c_dev = devm_kzalloc(&pdev->dev,sizeof(*i2c_dev), GFP_KERNEL);
	if(!i2c_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev,i2c_dev);  //struct device
	i2c_dev->dev = &pdev->dev;
	init_completion(&i2c_dev->completion);

	i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if(IS_ERR(i2c_dev->regs))
		return PTR_ERR(i2c_dev->regs);
	
	mclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(mclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(mclk),
				     "Could not get clock\n");

	i2c_dev->bus_clk = mfr_i2c_register_div(&pdev,mclk,i2c_dev);
	if(IS_ERR(i2c_dev->bus_clk))
	return dev_err_probe(&pdev,PTR_ERR(i2c_dev->bus_clk), "Could not register clock\n");

	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",&bus_clk_rate);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "Could not read clock-frequency property\n");
		bus_clk_rate = I2C_MAX_STANDARD_MODE_FREQ;
	}

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
