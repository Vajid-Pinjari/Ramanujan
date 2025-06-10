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
/*I2C registers values are assumed by ref bcm2711 */

//#define THIS_MODULE "mfr_i2c"
#define I2C_CONTROL_REG    0x00
#define I2C_STATUS_REG     0x04
#define I2C_DLEN_REG       0x08
#define I2C_SLAVE_ADDR_REG 0x0C
#define I2C_DATA_FIFO_REG  0x10
#define I2C_CLK_DIV_REG    0x14
#define I2C_DELAY_REG      0x18
#define I2C_CLK_STRETCH    0x1C
#define I2C_CLK_STRETCH_TIMEOUT 64

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

struct rjn_i2c_dev{
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

void rjn_fill_txfifo(struct rjn_i2c_dev *);
void rjn_drain_rxfifo(struct rjn_i2c_dev *);

//writel abstraction function to write data into reg directly
static inline void rjn_i2c_writel(struct rjn_i2c_dev *i2c_dev, u32 reg , u32 val)
{
    pr_info("writing into reg \n");
    writel(val,i2c_dev->regs + reg);
}

//readl abstraction function to read data into reg directly
static inline u32 rjn_i2c_readl(struct rjn_i2c_dev *i2c_dev, u32 reg)
{
    pr_info("Reading  from reg\n");
	return readl(i2c_dev->regs + reg);
}

void rjn_drain_rxfifo(struct rjn_i2c_dev *i2c_dev)
{
    u32 val;
    pr_info("i2c_rjn:receiving data from slave to fifo\n");
    while(i2c_dev->msg_buf_remaining)
    {
        /*wait until 1 byte of data will come into fifo*/
        do{
            val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
            if((val & I2C_S_CLKT) || (val & I2C_S_ERR))
            {
                pr_info("i2c_rjn:Exiting abnormally from drain_rxfifo\n");
                goto complete;
            }
        }
        while(!(val & I2C_S_RXD));
        
        *i2c_dev->msg_buf = rjn_i2c_readl(i2c_dev,I2C_DATA_FIFO_REG);
        i2c_dev->msg_buf++;
        i2c_dev->msg_buf_remaining--;
    }
    pr_info("i2c_rjn:normal exit from drain rxfifo\n");
    //Nack will generate after last byte read from controller
    complete:
    rjn_i2c_writel(i2c_dev,I2C_STATUS_REG,I2C_S_DONE);
}

void rjn_fill_txfifo(struct rjn_i2c_dev *i2c_dev)
{
    u32 val;
    pr_info("i2c_rjn:tranferring data to slave from fifo\n");
    while(i2c_dev->msg_buf_remaining)
    {
        do{
            val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
            if((val & I2C_S_CLKT) || (val & I2C_S_ERR))
            {
                pr_info("i2c_rjn:exiting abnormally from fill_txfifo\n");
                goto complete;
            }
        }
        while(!(val & I2C_S_TXD));
        rjn_i2c_writel(i2c_dev , I2C_DATA_FIFO_REG, *i2c_dev->msg_buf);
        i2c_dev->msg_buf++;
        i2c_dev->msg_buf_remaining--;
    }
    pr_info("i2c_rjn:exiting normally\n");
    //wait untill transfer done
    //Nack will generate after last byte read from controller
    //after all transaction completes clear status reg
    complete:
    rjn_i2c_writel(i2c_dev,I2C_STATUS_REG,I2C_S_DONE);
}

static void rjn_i2c_start_transfer(struct rjn_i2c_dev * i2c_dev)
{
    struct i2c_msg *msg = i2c_dev->curr_msg;

    pr_info("i2c_rjn:Start transfer\n");
    u32 c = I2C_C_ST |  I2C_C_EN;
    if(msg->flags & I2C_M_RD)   //checking for read flag from i2c core
    c |= I2C_C_READ;  //set read control bit

    rjn_i2c_writel(i2c_dev , I2C_DLEN_REG , msg->len);
    rjn_i2c_writel(i2c_dev, I2C_SLAVE_ADDR_REG, msg->addr);
    rjn_i2c_writel(i2c_dev, I2C_CONTROL_REG, c);
    pr_info("i2c_rjn:Start transfer completed\n");
}

static void rjn_i2c_finish_transfer(struct rjn_i2c_dev *i2c_dev)
{
    pr_info("i2c_rjn:Finish transfer\n");
	i2c_dev->curr_msg = NULL;
	i2c_dev->num_msgs = 0;

	i2c_dev->msg_buf = NULL;
	i2c_dev->msg_buf_remaining = 0;
    pr_info("i2c_rjn:Exiting....\n");
}

static int rjn_i2c_xfer(struct i2c_adapter *adap , struct i2c_msg msgs[],int num)
{
    struct rjn_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
    int val,err_flag=0;

    pr_info("i2c_rjn:in xfer\n");
    for(int i=0;i<num;i++)
    {
        i2c_dev->curr_msg = &msgs[i];
        rjn_i2c_start_transfer(i2c_dev);
        if(msgs[i].flags & I2C_M_RD)
        {
            //wait untill transfer done
            do{
                val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
            }
            while(!(val & I2C_S_DONE));
            //then drain rxfifo
            rjn_drain_rxfifo(i2c_dev);
        }
        else{
            rjn_fill_txfifo(i2c_dev);
            do{
                val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
            }
            while(!(val & I2C_S_DONE));
            }
    }
    rjn_i2c_finish_transfer(i2c_dev);

    val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
    if (val & I2C_S_ERR) {
    dev_err(i2c_dev->dev, "ACK error from slave\n");
    err_flag=1;
    }
    else if(val & I2C_S_CLKT)
    {
        dev_err(i2c_dev->dev, "Clock stretch timeout detected\n");
        err_flag=1;
    }

    rjn_i2c_writel(i2c_dev,I2C_STATUS_REG,I2C_S_DONE| I2C_S_ERR |I2C_S_CLKT);
    if(err_flag)
        return -EIO;
    else 
        return num;
}

static u32 rjn_i2c_func(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm rjn_i2c_algo = {
    .master_xfer = rjn_i2c_xfer,
    .functionality = rjn_i2c_func,
};

static const struct i2c_adapter_quirks rjn_i2c_quirks = {
    .flags = I2C_AQ_NO_CLK_STRETCH ,
};

static int init_i2c(struct rjn_i2c_dev *i2c_dev)
{
    pr_info("i2c_rjn:into init_i2c \n");
    unsigned long bus_clk_rate,CDIV,fedl,redl;
    //get the clock
    bus_clk_rate = clk_get_rate(i2c_dev->bus_clk);
    //calculate divider value for our clock
    pr_info("i2c_rjn:Iinitializing I2C\n");
    CDIV = bus_clk_rate / 1000000;  //1MHZ
    if( CDIV < I2C_CDIV_MIN || CDIV>I2C_CDIV_MAX) //cheking divider value is valid or not
    {
        return -EINVAL;
    }
    if(CDIV&1)   //Divider value always be even
    {
        CDIV++;
    }
    pr_info("i2c_rjn:setting i2c devider CDIV=%d\n",CDIV);
    
    /*
	 * Number of core clocks to wait after falling edge before
	 * outputting the next data bit.both FEDL and REDL
	 * can't be greater than CDIV/2.
	 */
	fedl = max(CDIV/16,1u);
	/*
	 * Number of core clocks to wait after rising edge before
	 * sampling the next incoming data bit.
	 */
	 redl = max(CDIV/4,1u);

    //Add clock divider to CDIV reg
    pr_info("i2c_rjn:setting redl=%d and fedl=%d\n",redl,fedl);
    rjn_i2c_writel(i2c_dev,I2C_CLK_DIV_REG,CDIV);
    //cfg fedl and redl value to DELAY reg
    rjn_i2c_writel(i2c_dev,I2C_DELAY_REG,fedl<<I2C_FEDL_SHIFT|redl<<I2C_REDL_SHIFT);
    //set clock stretch timeout
    rjn_i2c_writel(i2c_dev,I2C_CLK_STRETCH,0);
    //enable i2c
    rjn_i2c_writel(i2c_dev,I2C_CONTROL_REG,I2C_C_EN);
    pr_info("i2c_rjn:init_i2c completed\n");

    return 0;
}

static int rjn_i2c_probe(struct platform_device *pdev)
{
    struct rjn_i2c_dev *i2c_dev;
    int ret;
    struct i2c_adapter *adap;
    struct clk *mclk;
    unsigned int bus_clk_rate;

    // Step 1: Allocate and initialize memory for the device structure
    pr_info("i2c_rjn:In probe->Allocatimg memory");
    i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL );
    if(! i2c_dev)
    {
        pr_info("i2c_rjn:In probe->Allocatimg memory");
        return -ENOMEM;

    }

    // Step 2: Store the device structure for future reference
    platform_set_drvdata(pdev,i2c_dev);
    i2c_dev->dev = &pdev->dev;
    pr_info("i2c_rjn:In probe->set driver data \n");
	//pr_info("i2c_rjn:In probe->Initializing complextion");
    //init_completion(&i2c_dev->completion);

    // Step 3: Map hardware registers for I2C
    i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev,0,NULL);
    if(IS_ERR(i2c_dev->regs))
    {
        pr_info("i2c_rjn:failed in ioremap resourse\n");
        return PTR_ERR(i2c_dev->regs);
    }
    pr_info("i2c_rjn:In probe->got base address %p\n",i2c_dev->regs);

    // Step 4: Get the main clock for the device
    mclk = devm_clk_get(&pdev->dev, NULL);
    if(IS_ERR(mclk))
    {
        pr_info("i2c_rjn:In probe->get mclk failed\n);
        return dev_err_probe(&pdev->dev,PTR_ERR(mclk),"Could not get clock\n");
    }
    pr_info("i2c_rjn:In probe->get mclk sucess\n);
    i2c_dev->bus_clk = mclk; //Assign the clock handle to the struct

    // Step 5: Read the clock frequency from the device tree
    //removed reading from DT and added statically.
    bus_clk_rate = I2C_MAX_FAST_MODE_PLUS_FREQ;
    
    // Step 6: Set the exclusive clock rate for the I2C bus
	ret = clk_set_rate_exclusive(i2c_dev->bus_clk,bus_clk_rate);
    if(ret<0)
        return dev_err_probe(&pdev->dev , ret , "Could not set clock frequency ");
    pr_info("set rate exclusive sucess\n");
    // Step 7: Prepare and enable the clock for use
    ret = clk_prepare_enable(i2c_dev->bus_clk);
    if(ret)
    {
        dev_err(&pdev->dev, "Couldn't prepare clock");
        goto err_put_exclusive_rate;
    }
    pr_info("i2c_rjn:In probe->prepared and enabled clock"\n);

    // Step 8:init i2c
    ret = init_i2c(i2c_dev);
    if(ret)
    {
        dev_err(&pdev->dev,"initialization error\n");
        goto err_disable_unprepare_clk;
    }
    pr_info("i2c_rjn:init_i2c sucess\n");

    // Step 9: Initialize the I2C adapter structure
    adap = &i2c_dev->adapter;
    // Step 10: Set adapter metadata (name, owner, algorithm, parent, etc.)
    i2c_set_adapdata(adap,i2c_dev);
    adap->owner = THIS_MODULE;
    adap->class = I2C_CLASS_HWMON; //(H/w monitoring devices) //I2C_CLASS_DEPRECATED; for no class
    snprintf(adap->name, sizeof(adap->name), "mfr (%s)", of_node_full_name(pdev->dev.of_node));
    adap->algo = &rjn_i2c_algo;
    adap->dev.parent = &pdev->dev;

    // Step 11: Register the adapter with the I2C subsystem
    adap->dev.of_node = pdev->dev.of_node;
    adap->quirks = of_device_get_match_data(&pdev->dev);	
    
    // Step 12: Add the adapter to the I2C framework
    pr_info("i2c_rjn:In probe->registering adapter \n");
    ret = i2c_add_adapter(adap);
    if(ret){
        goto err_disable_unprepare_clk;
    }
    pr_info("i2c_rjn:In probe->Adapter added sucessfully\n");

    return 0;
	
    // Error Handling: Free IRQ, disable clock, and cleanup on failure
    err_disable_unprepare_clk:
	    clk_disable_unprepare(i2c_dev->bus_clk);
    err_put_exclusive_rate:
        clk_rate_exclusive_put(i2c_dev->bus_clk);

    return ret;
}

static void rjn_i2c_remove(struct platform_device *pdev)
{
    struct rjn_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	clk_rate_exclusive_put(i2c_dev->bus_clk);
	clk_disable_unprepare(i2c_dev->bus_clk);

	i2c_del_adapter(&i2c_dev->adapter);

    //return 0;
}

static const struct of_device_id rjn_i2c_of_match[]={
	{ 
		.name = "rjn-i2c",
		.type = "i2c",
		.compatible = "mirafra,rjn-i2c",
	},
	{ .compatible = "mirafra,rjn-i2c" , .data = &rjn_i2c_quirks },
	{},
};

MODULE_DEVICE_TABLE(of, rjn_i2c_of_match);

static struct platform_driver i2c_platform_driver = {
.probe = rjn_i2c_probe,
.remove = rjn_i2c_remove,
.driver = {
	.name = "rjn-i2c",
	.of_match_table = rjn_i2c_of_match,
},
};

module_platform_driver(i2c_platform_driver);

/*Driver Metadata*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramanujan SOC :Mirafra");
MODULE_DESCRIPTION("Platform :I2C Driver for RAMANUJAN");