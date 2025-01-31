/* Writing a I2C driver for Ramanujan SOC*/

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
/*I2C registers */

//#define THIS_MODULE "mfr_i2c"
#define I2C_CONTROL_REG    0x00
#define I2C_STATUS_REG     0x04
#define I2C_DLEN_REG       0x08
#define I2C_SLAVE_ADDR_REG 0x0C
#define I2C_DATA_FIFO_REG  0x10
#define I2C_CLK_DIV_REG    0x14
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
#define I2C_CDIV_MIN	0x0002
#define I2C_CDIV_MAX	0xFFFE


struct mfr_i2c_dev{
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


#define to_clk_mfr_i2c(_hw)  container_of(_hw , struct clk_mfr_i2c, hw)
struct clk_mfr_i2c{
    struct clk_hw hw;
    struct mfr_i2c_dev *i2c_dev;
};

static inline void mfr_i2c_writel(struct mfr_i2c_dev *i2c_dev, u32 reg , u32 val)
{
    writel(val,i2c_dev->regs + reg);
}

static inline u32 mfr_i2c_readl(struct mfr_i2c_dev *i2c_dev, u32 reg)
{
	return readl(i2c_dev->regs + reg);
}


static int clk_mfr_i2c_calc_divider(unsigned long rate,unsigned long parent_rate)
{
    //set the divider {approximately = pr/r}
    u32 divider = DIV_ROUND_UP( parent_rate , rate );
    
    //to roundup for even num(as per datasheet)
    if(divider & 1)
    divider++;

    //checkking the devider value is valid or not 
    if((divider < I2C_CDIV_MIN) || (divider > I2C_CDIV_MAX))
    return -EINVAL;
    
    return divider;
}


static int clk_mfr_i2c_set_rate(struct clk_hw *hw, unsigned long rate,unsigned long parent_rate)
{
    struct clk_mfr_i2c *div = to_clk_mfr_i2c(hw);
    unsigned int redl,fedl;
    unsigned int divider = clk_mfr_i2c_calc_divider(rate,parent_rate);
    if(divider == -EINVAL)
        return -EINVAL;

    mfr_i2c_writel(div->i2c_dev, I2C_CLK_DIV_REG, divider);

    //scaling factor after transition of falling edge
    fedl = max( divider/16 ,1u );  
    //scaling factor after transition of rising edge
    redl = max(divider/4 ,1u);

    mfr_i2c_writel(div->i2c_dev , I2C_DATA_DELAY_REG , (fedl<<I2C_FEDL_SHIFT) | (redl<<I2C_REDL_SHIFT));
    

    return 0;
}

static long clk_mfr_i2c_round_rate(struct clk_hw *hw , unsigned long rate,unsigned long *parent_rate)
{
    u32 divider = clk_mfr_i2c_calc_divider(rate,*parent_rate);

    return DIV_ROUND_UP(*parent_rate , divider);
}

static unsigned long clk_mfr_i2c_recalc_rate(struct clk_hw  *hw, unsigned long parent_rate)
{
    struct clk_mfr_i2c *div = to_clk_mfr_i2c(hw);
    u32 divider = mfr_i2c_readl( div->i2c_dev , I2C_CLK_DIV_REG );

    return DIV_ROUND_UP(parent_rate, divider);
}

static const struct clk_ops clk_mfr_i2c_ops = {
    .set_rate =  clk_mfr_i2c_set_rate,
    .round_rate = clk_mfr_i2c_round_rate,
    .recalc_rate = clk_mfr_i2c_recalc_rate,
};

static struct clk *mfr_i2c_register_div( struct device *dev ,
                                        struct clk *mclk ,
                                        struct mfr_i2c_dev *i2c_dev)
{
    struct clk_init_data init;
    struct clk_mfr_i2c *prev;
    char name[32];
    const char *mclk_name;

// Step 1: Generate clock name based on device name & Get the name of the parent clock
snprintf(name,sizeof(name), "%s_div" ,dev_name(dev));

mclk_name = __clk_get_name(mclk);

// Step 2: Initialize clock settings (operations, name, parent, etc.)
init.ops = &clk_mfr_i2c_ops;
init.name = name;
init.parent_names = (const char * []){ mclk_name };
init.num_parents = 1;
init.flags = 0;

// Step 3: Allocate memory for private data structure
prev = devm_kzalloc(dev, sizeof(struct clk_mfr_i2c),GFP_KERNEL);
if(prev == NULL )
    return ERR_PTR(-ENOMEM);

// Step 4: Set up the clock hardware structure with initialization data
prev->hw.init = &init;
prev->i2c_dev = i2c_dev ;

// Step 5: Register the clock device with the system
clk_hw_register_clkdev(&prev->hw , "div",dev_name(dev));

// Step 6: Register the clock with the device and return the result
    return devm_clk_register(dev, &prev->hw);
}



static int mfr_i2c_probe(struct platform_device *pdev)
{
    struct mfr_i2c_dev *i2c_dev;
    int ret;
    struct i2c_adapter *adap;
    struct clk *mclk;
    u32 bus_clk_rate;

    // Step 1: Allocate and initialize memory for the device structure
    i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL );
    if(! i2c_dev)
	    return -ENOMEM;	

    // Step 2: Store the device structure for future reference
    platform_set_drvdata(pdev,i2c_dev);
    i2c_dev->dev = &pdev->dev;
	
    // Step 3: Initialize device-specific completion mechanism
    init_completion(&i2c_dev->completion);
	
    // Step 4: Map hardware registers for I2C
    i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev,0,NULL);
    if(IS_ERR(i2c_dev->regs))
        return PTR_ERR(i2c_dev->regs);

    // Step 5: Get the main clock for the device
    mclk = devm_clk_get(&pdev->dev, NULL);
    if(IS_ERR(mclk))
        return dev_err_probe(&pdev->dev,PTR_ERR(mclk),"Could not get clock\n");

    // Step 6: Register and configure the bus clock divider
	i2c_dev->bus_clk = mfr_i2c_register_div(&pdev->dev, mclk ,i2c_dev);
    if (IS_ERR(i2c_dev->bus_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(i2c_dev->bus_clk),
				     "Could not register clock\n");

    // Step 7: Read the clock frequency from the device tree
    ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",&bus_clk_rate);
    if(ret<0)
    {
        dev_warn(&pdev->dev, "Could not read clock-frequency property\n");
         //(default to standard I2C frequency if unavailable)
        bus_clk_rate = I2C_MAX_STANDARD_MODE_FREQ;
	}
    
    // Step 8: Set the exclusive clock rate for the I2C bus
	ret = clk_set_rate_exclusive(i2c_dev->bus_clk,bus_clk_rate);
    if(ret<0)
        return dev_err_probe(&pdev->dev , ret , "Could not set clock frequency ");
    // Step 9: Prepare and enable the clock for use
    ret = clk_prepare_enable(i2c_dev->bus_clk);
    if(ret)
    {
        dev_err(&pdev->dev, "Couldn't prepare clock");
        goto err_disable_unprepare_clk;
    }

    // Step 10: Retrieve the interrupt line for the device
	
    // Step 11: Request an IRQ and set the ISR for handling interrupts

    // Step 12: Initialize the I2C adapter structure
	
    // Step 13: Set adapter metadata (name, owner, algorithm, parent, etc.)
	
    // Step 14: Register the adapter with the I2C subsystem

    // Step 15: Configure hardware registers (disable clock stretching timeout)
	
    // Step 16: Write default configurations to hardware registers

    // Step 17: Add the adapter to the I2C framework
	
    // Error Handling: Free IRQ, disable clock, and cleanup on failure
err_disable_unprepare_clk:
	clk_disable_unprepare(i2c_dev->bus_clk);

    return ret;
}

static int mfr_i2c_remove(struct platform_device *pdev)
{
	return 0;
}


static const struct of_device_id mfr_i2c_of_match[]={
	{ 
		.name = "mfr_i2c",
		.type = "i2c",
		.compatible = "mfr_i2c0",
	},
/*	{ .compatible = "mfr_i2c1" , .data = &mfr_i2c_quirks },*/
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
MODULE_AUTHOR("Ramanujan SOC :Mirafra");
MODULE_DESCRIPTION("I2C Driver for RAMANUJAN");

/*
In Falling edge delay shift register 
we are deviding divider / 16
fedl = divider/16 ;(2^n);
& it should be greater than or equal to 1
to take 16 as a scaling factor is there any specific parameter ?
*/
