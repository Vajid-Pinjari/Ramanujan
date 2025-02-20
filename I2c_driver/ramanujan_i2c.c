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
#define I2C_S_LEN	BIT(10) /* Fake bit for SW error reporting */

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

void mfr_fill_txfifo(struct mfr_i2c_dev *);
void mfr_drain_rxfifo(struct mfr_i2c_dev *);
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

void mfr_drain_rxfifo(struct mfr_i2c_dev *i2c_dev)
{
    u32 val;
    while(i2c_dev->msg_buf_remaining)
    {
        val = mfr_i2c_readl(i2c_dev, I2C_STATUS_REG);
        if(!(val & I2C_S_RXD))
        break;
        *i2c_dev->msg_buf = mfr_i2c_readl(i2c_dev,I2C_DATA_FIFO_REG);
        i2c_dev->msg_buf++;
        i2c_dev->msg_buf_remaining--;
    }
}

void mfr_fill_txfifo(struct mfr_i2c_dev *i2c_dev)
{
    u32 val;
    while(i2c_dev->msg_buf_remaining)
    {
        val = mfr_i2c_readl(i2c_dev, I2C_STATUS_REG);
        if(!(val & I2C_S_TXD))
            break;
        mfr_i2c_writel(i2c_dev , I2C_DATA_FIFO_REG, *i2c_dev->msg_buf);
        i2c_dev->msg_buf++;
        i2c_dev->msg_buf_remaining--;
    }
}

static void mfr_i2c_start_transfer(struct mfr_i2c_dev * i2c_dev)
{
    u32 c = I2C_C_ST |  I2C_C_EN;
    struct i2c_msg *msg = i2c_dev->curr_msg;
    bool last_msg = (i2c_dev->num_msgs ==1 );

    if(!i2c_dev->num_msgs)
    return;

    i2c_dev->num_msgs--;
    i2c_dev->msg_buf = msg->buf;
    i2c_dev->msg_buf_remaining = msg->len;

    if(msg->flags & I2C_M_RD)
    c |= I2C_C_READ | I2C_C_INTR;
    else 
    c |= I2C_C_INTT ;
    if(last_msg)
    c |= I2C_C_INTD;

    mfr_i2c_writel(i2c_dev, I2C_SLAVE_ADDR_REG, msg->addr);
    mfr_i2c_writel(i2c_dev , I2C_DLEN_REG , msg->len);
    mfr_i2c_writel(i2c_dev, I2C_CONTROL_REG, c);

}



static void mfr_i2c_finish_transfer(struct mfr_i2c_dev *i2c_dev)
{
	i2c_dev->curr_msg = NULL;
	i2c_dev->num_msgs = 0;

	i2c_dev->msg_buf = NULL;
	i2c_dev->msg_buf_remaining = 0;
}

static irqreturn_t mfr_i2c_isr(int this_irq, void *data)
{ 
    struct mfr_i2c_dev *i2c_dev = data;
    u32 val , err;
    
    val = mfr_i2c_readl(i2c_dev,I2C_STATUS_REG);

    err = val & (I2C_S_CLKT | I2C_S_ERR);
    if(err)
    {
        i2c_dev->msg_err = err;
        goto complete;
    }

    if(val & I2C_S_DONE )
    {
        if(!i2c_dev->curr_msg)
        {
            dev_err(i2c_dev->dev , "Got unexpected interrupt(from firmware?)\n");
        }
        else if(i2c_dev->curr_msg->flags & I2C_M_RD )
        {
            mfr_drain_rxfifo(i2c_dev);
            val = mfr_i2c_readl(i2c_dev, I2C_STATUS_REG);
        }

        if((val & I2C_S_RXD) || i2c_dev->msg_buf_remaining)
            i2c_dev->msg_err = I2C_S_LEN;
        else 
            i2c_dev->msg_err =0 ;
        
        goto complete;
    }

    if(val & I2C_S_TXW){
        if(!i2c_dev->msg_buf_remaining){
            i2c_dev->msg_err = val | I2C_S_LEN;
            goto complete;
        }

        mfr_fill_txfifo(i2c_dev);

        if(i2c_dev->num_msgs && !i2c_dev->msg_buf_remaining)
        {
            i2c_dev->curr_msg++;
            mfr_i2c_start_transfer(i2c_dev);
        }

        return IRQ_HANDLED;
    }

    if(val & I2C_S_RXR )
    {
        if(!i2c_dev->msg_buf_remaining)
        {
            i2c_dev->msg_err = val | I2C_S_LEN ;
            goto complete;
        }
        mfr_drain_rxfifo(i2c_dev);
        return IRQ_HANDLED;
    }
    return IRQ_NONE;

complete : mfr_i2c_writel(i2c_dev, I2C_CONTROL_REG,I2C_C_CLEAR);
           mfr_i2c_writel(i2c_dev, I2C_STATUS_REG, I2C_S_CLKT |
           I2C_S_ERR | I2C_S_DONE);
           complete(&i2c_dev->completion);

           return IRQ_HANDLED;
}

static int mfr_i2c_xfer(struct i2c_adapter *adap , struct i2c_msg msgs[],int num)
{
    struct mfr_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
    unsigned long time_left;
    int i;

    /* 1: Check if multiple read messages are requested 
	         Only a single read message is supported and it must be the last */

    for(i=0;i<(num-1);i++)
    {
        if(msgs[i].flags & I2C_M_RD)
        {
            dev_warn_once(i2c_dev->dev, "only one read message supported, has to be last\n");
            return -EOPNOTSUPP;   // Return error if unsupported request is found
        }
    }

    /* Step 2: Initialize transfer parameters */
    i2c_dev->curr_msg = msgs;   // Store the message array
    i2c_dev->num_msgs = num;    // Store the number of messages
    reinit_completion(&i2c_dev->completion);  // Reinitialize the completion structure

	/* Step 3: Start the I2C transfer */
    mfr_i2c_start_transfer(i2c_dev);

    /* Step 4: Wait for transfer completion with timeout */
    time_left = wait_for_completion_timeout(&i2c_dev->completion,adap->timeout);
    
    /*Step 5: Finish transfer*/
    mfr_i2c_finish_transfer(i2c_dev);

    /* Step 6: Check if transfer timed out */
    if(!time_left)
    {
        mfr_i2c_writel(i2c_dev ,I2C_CONTROL_REG, I2C_C_CLEAR);
        return -ETIMEDOUT;
    }

    /* Step 7: Check for errors during transfer */
    if(!i2c_dev->msg_err)
        return num;
    /* Log the error message if transfer failed */
    dev_dbg(i2c_dev->dev, "i2c_transfer_failed : %x\n",i2c_dev->msg_err);

    /* Step 8: Return appropriate error code based on failure type */
    if(i2c_dev->msg_err & I2C_S_ERR)
        return -EREMOTEIO;

    return -EIO;
}

static u32 mfr_i2c_func(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm mfr_i2c_algo = {
    .master_xfer = mfr_i2c_xfer,
    .functionality = mfr_i2c_func,
};

static const struct i2c_adapter_quirks mfr_i2c_quirks = {
    .flags = I2C_AQ_NO_CLK_STRETCH,
};


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
    i2c_dev->irq = platform_get_irq(pdev,0);
    if(i2c_dev->irq < 0)
    {
        ret = i2c_dev->irq;
        goto err_disable_unprepare_clk;
    }
	
    // Step 11: Request an IRQ and set the ISR for handling interrupts
    ret = request_irq(i2c_dev->irq, mfr_i2c_isr, IRQF_SHARED, dev_name(&pdev->dev), i2c_dev);
    if(ret)
    {
        dev_err(&pdev->dev, "Could not request IRQ\n");
        goto err_disable_unprepare_clk;
    }

    // Step 12: Initialize the I2C adapter structure
    adap = &i2c_dev->adapter;
    // Step 13: Set adapter metadata (name, owner, algorithm, parent, etc.)
    i2c_set_adapdata(adap,i2c_dev);
    adap->owner = THIS_MODULE;
    adap->class = I2C_CLASS_DEPRECATED;
    snprintf(adap->name, sizeof(adap->name), "mfr (%s)", of_node_full_name(pdev->dev.of_node));
    adap->algo = &mfr_i2c_algo;
    adap->dev.parent = &pdev->dev;

    // Step 14: Register the adapter with the I2C subsystem
    adap->dev.of_node = pdev->dev.of_node;
    adap->quirks = of_device_get_match_data(&pdev->dev);	
    
    // Step 15: Configure hardware registers (disable clock stretching timeout)
    mfr_i2c_writel(i2c_dev , I2C_CLK_STRETCH, 0);
    // Step 16: Write default configurations to hardware registers
    mfr_i2c_writel(i2c_dev , I2C_CONTROL_REG,0);

    // Step 17: Add the adapter to the I2C framework
    ret = i2c_add_adapter(adap);
    if(ret){
        goto err_free_irq;
    }

    return 0;
	
    // Error Handling: Free IRQ, disable clock, and cleanup on failure
err_free_irq: 
    free_irq(i2c_dev->irq,i2c_dev);
err_disable_unprepare_clk:
	clk_disable_unprepare(i2c_dev->bus_clk);
err_put_exclusive_rate:
    clk_rate_exclusive_put(i2c_dev->bus_clk);

    return ret;
}

static int mfr_i2c_remove(struct platform_device *pdev)
{
	
    struct mfr_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	clk_rate_exclusive_put(i2c_dev->bus_clk);
	clk_disable_unprepare(i2c_dev->bus_clk);

	free_irq(i2c_dev->irq, i2c_dev);
	i2c_del_adapter(&i2c_dev->adapter);

    return 0;
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
MODULE_AUTHOR("Ramanujan SOC :Mirafra");
MODULE_DESCRIPTION("Platform :I2C Driver for RAMANUJAN");