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

/* I2C registers values are assumed by ref bcm2711 */
#define I2C_CONTROL_REG    0x00
#define I2C_STATUS_REG     0x04
#define I2C_DLEN_REG       0x08
#define I2C_SLAVE_ADDR_REG 0x0C
#define I2C_DATA_FIFO_REG  0x10
#define I2C_CLK_DIV_REG    0x14
#define I2C_DELAY_REG      0x18
#define I2C_CLK_STRETCH    0x1C
#define I2C_CLK_STRETCH_TIMEOUT 64

// I2C control Register bits
#define I2C_C_READ  BIT(0) // Read Bit
#define I2C_C_CLEAR BIT(4) // BIT 4 & 5
#define I2C_C_ST    BIT(7) // Start Xfer
#define I2C_C_INTD  BIT(8) // Int on Done
#define I2C_C_INTT  BIT(9) // Int on Xfer
#define I2C_C_INTR  BIT(10) // Int on RX
#define I2C_C_EN    BIT(15) // Enable I2C

// I2C Status Register
#define I2C_S_TA        BIT(0) // Xfer Active
#define I2C_S_DONE      BIT(1) // Xfer Done
#define I2C_S_TXW       BIT(2) // FIFO needs writing
#define I2C_S_RXR       BIT(3) // FIFO needs Reading
#define I2C_S_TXD       BIT(4) // Accept Data
#define I2C_S_RXD       BIT(5) // Contains Data
#define I2C_S_TXE       BIT(6) // Empty
#define I2C_S_RXF       BIT(7) // Full
#define I2C_S_ERR       BIT(8) // ACK err
#define I2C_S_CLKT      BIT(9) // Clock Stretch Timeout
#define I2C_S_LEN       BIT(10) // Fake bit for SW error reporting

// I2C DEL(Delay) Register
#define I2C_FEDL_SHIFT  16 // Falling Edge Delay
#define I2C_REDL_SHIFT  0  // Rising Edge Delay

// I2C CDIV Register
#define I2C_CDIV_MIN    0x0002
#define I2C_CDIV_MAX    0xFFFE

struct rjn_i2c_dev {
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

// writel abstraction function to write data into reg directly
static inline void rjn_i2c_writel(struct rjn_i2c_dev *i2c_dev, u32 reg, u32 val)
{
    writel(val, i2c_dev->regs + reg);
    dev_dbg(i2c_dev->dev, "Write reg 0x%x = 0x%x\n", reg, val);
}

// readl abstraction function to read data into reg directly
static inline u32 rjn_i2c_readl(struct rjn_i2c_dev *i2c_dev, u32 reg)
{
    u32 val = readl(i2c_dev->regs + reg);
    dev_dbg(i2c_dev->dev, "Read reg 0x%x = 0x%x\n", reg, val);
    return val;
}

void rjn_drain_rxfifo(struct rjn_i2c_dev *i2c_dev)
{
    u32 val;
    while (i2c_dev->msg_buf_remaining) {
        val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
        if (!(val & I2C_S_RXD))
            break;

        *i2c_dev->msg_buf = rjn_i2c_readl(i2c_dev, I2C_DATA_FIFO_REG);
        i2c_dev->msg_buf++;
        i2c_dev->msg_buf_remaining--;
    }
}

void rjn_fill_txfifo(struct rjn_i2c_dev *i2c_dev)
{
    u32 val;
    while (i2c_dev->msg_buf_remaining) {
        val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
        if (!(val & I2C_S_TXD))
            break;
        rjn_i2c_writel(i2c_dev, I2C_DATA_FIFO_REG, *i2c_dev->msg_buf);
        i2c_dev->msg_buf++;
        i2c_dev->msg_buf_remaining--;
    }
}

static void rjn_i2c_start_transfer(struct rjn_i2c_dev *i2c_dev)
{
    __u32 c = I2C_C_ST | I2C_C_EN;
    struct i2c_msg *msg = i2c_dev->curr_msg;
    bool last_msg = (i2c_dev->num_msgs == 1);
    if (!i2c_dev->num_msgs)
        return;

    dev_info(i2c_dev->dev, "Starting transfer: addr=0x%x, len=%d, read=%d\n",
             msg->addr, msg->len, !!(msg->flags & I2C_M_RD));

    i2c_dev->num_msgs--;
    i2c_dev->msg_buf = msg->buf;
    i2c_dev->msg_buf_remaining = msg->len;

    if (msg->flags & I2C_M_RD)
        c |= I2C_C_READ | I2C_C_INTR;
    else
        c |= I2C_C_INTT;

    if (last_msg)
        c |= I2C_C_INTD;

    rjn_i2c_writel(i2c_dev, I2C_DLEN_REG, msg->len);
    rjn_i2c_writel(i2c_dev, I2C_SLAVE_ADDR_REG, msg->addr);
    rjn_i2c_writel(i2c_dev, I2C_CONTROL_REG, c);
}

static void rjn_i2c_finish_transfer(struct rjn_i2c_dev *i2c_dev)
{
    i2c_dev->curr_msg = NULL;
    i2c_dev->num_msgs = 0;
    i2c_dev->msg_buf = NULL;
    i2c_dev->msg_buf_remaining = 0;
}

static int rjn_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
    struct rjn_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
    unsigned long time_left;

    dev_info(i2c_dev->dev, "Starting I2C transfer, num_msgs: %d\n", num);

    i2c_dev->curr_msg = msgs;
    i2c_dev->num_msgs = num;
    reinit_completion(&i2c_dev->completion);

    rjn_i2c_start_transfer(i2c_dev);

    time_left = wait_for_completion_timeout(&i2c_dev->completion, adap->timeout);

    rjn_i2c_finish_transfer(i2c_dev);

    if (!time_left) {
        rjn_i2c_writel(i2c_dev, I2C_CONTROL_REG, I2C_C_CLEAR);
        dev_err(i2c_dev->dev, "Timeout error detected\n");
        return -ETIMEDOUT;
    }

    if (!i2c_dev->msg_err) {
        dev_info(i2c_dev->dev, "Transfer completed successfully\n");
        return num;
    }

    dev_info(i2c_dev->dev, "I2C transfer failed: 0x%x\n", i2c_dev->msg_err);

    if (i2c_dev->msg_err & I2C_S_CLKT) {
        dev_err(i2c_dev->dev, "Clock stretch timeout detected\n");
        return -EREMOTEIO;
    } else if (i2c_dev->msg_err & I2C_S_ERR) {
        dev_err(i2c_dev->dev, "I2C ACK error detected\n");
        return -EIO;
    } else if (i2c_dev->msg_err & I2C_S_LEN) {
        dev_err(i2c_dev->dev, "I2C length error detected\n");
        return -EIO;
    }

    return -EIO;
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
    .flags = 0, // Allow clock stretching
};

static int init_i2c(struct rjn_i2c_dev *i2c_dev, unsigned long bus_clk_rate)
{
    unsigned long CDIV, fedl, redl;
    u32 status;

    dev_info(i2c_dev->dev, "Initializing I2C with bus clock rate: %lu Hz\n", bus_clk_rate);

    CDIV = clk_get_rate(i2c_dev->bus_clk) / bus_clk_rate;
    if (CDIV < I2C_CDIV_MIN || CDIV > I2C_CDIV_MAX) {
        dev_err(i2c_dev->dev, "Invalid clock divider: %lu\n", CDIV);
        return -EINVAL;
    }
    if (CDIV & 1) {
        CDIV++;
    }

    fedl = CDIV / 8;
    redl = CDIV / 8;

    rjn_i2c_writel(i2c_dev, I2C_CLK_DIV_REG, CDIV);
    rjn_i2c_writel(i2c_dev, I2C_DELAY_REG, fedl << I2C_FEDL_SHIFT | redl << I2C_REDL_SHIFT);
    rjn_i2c_writel(i2c_dev, I2C_CLK_STRETCH, I2C_CLK_STRETCH_TIMEOUT);
    rjn_i2c_writel(i2c_dev, I2C_CONTROL_REG, I2C_C_EN);

    // Verify I2C controller status
    status = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
    dev_info(i2c_dev->dev, "I2C status after init: 0x%x\n", status);
    if (status & (I2C_S_ERR | I2C_S_CLKT)) {
        dev_err(i2c_dev->dev, "I2C controller in error state after init\n");
        return -EIO;
    }

    dev_info(i2c_dev->dev, "I2C initialized with CDIV: %lu, FEDL: %lu, REDL: %lu\n", CDIV, fedl, redl);

    return 0;
}

static irqreturn_t rjn_i2c_isr(int irq, void *data)
{
    int val;
    struct rjn_i2c_dev *i2c_dev = data;
    val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);

    dev_info(i2c_dev->dev, "Interrupt triggered, status: 0x%x\n", val);

    if (val & (I2C_S_CLKT | I2C_S_ERR)) {
        i2c_dev->msg_err = (val & (I2C_S_CLKT | I2C_S_ERR));
        goto complete;
    }

    if (val & I2C_S_DONE) {
        if (!i2c_dev->curr_msg) {
            dev_info(i2c_dev->dev, "IRQ after transfer complete — likely harmless, ignoring.\n");
            return IRQ_HANDLED;
        } else if (i2c_dev->curr_msg->flags & I2C_M_RD) {
            rjn_drain_rxfifo(i2c_dev);
            val = rjn_i2c_readl(i2c_dev, I2C_STATUS_REG);
            if (val & I2C_S_RXD || i2c_dev->msg_buf_remaining)
                i2c_dev->msg_err = I2C_S_LEN;
            else
                i2c_dev->msg_err = 0;
        }
        goto complete;
    }

    if (val & I2C_S_TXW) {
        if (!i2c_dev->msg_buf_remaining) {
            i2c_dev->msg_err = I2C_S_LEN;
            goto complete;
        }

        rjn_fill_txfifo(i2c_dev);

        if (!i2c_dev->msg_buf_remaining && i2c_dev->num_msgs) {
            i2c_dev->curr_msg++;
            rjn_i2c_start_transfer(i2c_dev);
        }
        return IRQ_HANDLED;
    }

    if (val & I2C_S_RXR) {
        if (!i2c_dev->msg_buf_remaining) {
            i2c_dev->msg_err = I2C_S_LEN;
            goto complete;
        }

        rjn_drain_rxfifo(i2c_dev);
        return IRQ_HANDLED;
    }
    return IRQ_NONE;

complete:
    rjn_i2c_writel(i2c_dev, I2C_CONTROL_REG, I2C_C_CLEAR);
    complete(&i2c_dev->completion);
    return IRQ_HANDLED;
}

static int rjn_i2c_probe(struct platform_device *pdev)
{
    struct rjn_i2c_dev *i2c_dev;
    int ret;
    struct i2c_adapter *adap;
    struct clk *mclk;
    unsigned int bus_clk_rate;

    dev_info(&pdev->dev, "rjn_i2c: probe called\n");

    if (!pdev->dev.of_node) {
        dev_err(&pdev->dev, "No device tree node found\n");
        return -ENODEV;
    }

    i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
    if (!i2c_dev)
        return -ENOMEM;

    dev_info(&pdev->dev, "rjn_i2c: memory allocated for device structure\n");

    init_completion(&i2c_dev->completion);
    dev_info(&pdev->dev,"rjn_i2c:initialized completion mechanism");

    platform_set_drvdata(pdev, i2c_dev);
    i2c_dev->dev = &pdev->dev;

    i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
    if (IS_ERR(i2c_dev->regs))
        return dev_err_probe(&pdev->dev, PTR_ERR(i2c_dev->regs), "Failed to map registers\n");
    dev_info(&pdev->dev, "rjn_i2c: Got base address: 0x%px\n", i2c_dev->regs);

    mclk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(mclk))
        return dev_err_probe(&pdev->dev, PTR_ERR(mclk), "Could not get clock\n");
    dev_info(&pdev->dev, "rjn_i2c: got clock from DT\n");

    i2c_dev->bus_clk = mclk;

    ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &bus_clk_rate);
    if (ret < 0) {
        dev_warn(&pdev->dev, "Could not read clock-frequency property, defaulting to 100 kHz\n");
        bus_clk_rate = I2C_MAX_STANDARD_MODE_FREQ; // 100 kHz
    }
    dev_info(&pdev->dev, "rjn_i2c: setting clock frequency: %u Hz\n", bus_clk_rate);

    ret = clk_set_rate_exclusive(i2c_dev->bus_clk, bus_clk_rate);
    if (ret < 0)
        return dev_err_probe(&pdev->dev, ret, "Could not set clock frequency\n");
    dev_info(&pdev->dev, "rjn_i2c: set clock frequency exclusively\n");

    ret = clk_prepare_enable(i2c_dev->bus_clk);
    if (ret) {
        dev_err(&pdev->dev, "Couldn't prepare clock\n");
        goto err_put_exclusive_rate;
    }
    dev_info(&pdev->dev, "rjn_i2c: prepared and enabled clock\n");

    i2c_dev->irq = platform_get_irq(pdev, 0);
    if (i2c_dev->irq < 0) {
        dev_err(&pdev->dev, "Failed to get IRQ from DT\n");
        goto err_disable_unprepare_clk;
    }
    dev_info(&pdev->dev, "rjn_i2c: Got IRQ line: %d\n", i2c_dev->irq);

    ret = devm_request_irq(&pdev->dev, i2c_dev->irq, rjn_i2c_isr,
                           IRQF_SHARED, dev_name(&pdev->dev), i2c_dev);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to request IRQ for ISR\n");
        goto err_disable_unprepare_clk;
    }
    dev_info(&pdev->dev, "rjn_i2c: Requested IRQ\n");

    ret = init_i2c(i2c_dev, bus_clk_rate);
    if (ret) {
        dev_err(&pdev->dev, "I2C initialization error\n");
        goto err_disable_unprepare_clk;
    }
    dev_info(&pdev->dev, "rjn_i2c: Initialized I2C controller\n");

    adap = &i2c_dev->adapter;
    i2c_set_adapdata(adap, i2c_dev);
    adap->owner = THIS_MODULE;
    adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
    adap->timeout = msecs_to_jiffies(1000);
    adap->retries = 3; // Set default retries
    strscpy(adap->name, "rjn-i2c", sizeof(adap->name));
    adap->algo = &rjn_i2c_algo;
    adap->dev.parent = &pdev->dev;
    adap->dev.of_node = pdev->dev.of_node;
    adap->quirks = of_device_get_match_data(&pdev->dev);

    dev_info(&pdev->dev, "rjn_i2c: Adding adapter: %s\n", adap->name);
    ret = i2c_add_adapter(adap);
    if (ret) {
        dev_err(&pdev->dev, "Failed to add I2C adapter: %d\n", ret);
        goto err_disable_unprepare_clk;
    }
    dev_info(&pdev->dev, "rjn_i2c: Added adapter, bus number: %d\n", adap->nr);

    return 0;

err_disable_unprepare_clk:
    clk_disable_unprepare(i2c_dev->bus_clk);
err_put_exclusive_rate:
    clk_rate_exclusive_put(i2c_dev->bus_clk);

    return ret;
}

static int rjn_i2c_remove(struct platform_device *pdev)
{
    struct rjn_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "rjn_i2c: Removing adapter\n");
    clk_rate_exclusive_put(i2c_dev->bus_clk);
    clk_disable_unprepare(i2c_dev->bus_clk);
    i2c_del_adapter(&i2c_dev->adapter);

    return 0;
}

static const struct of_device_id rjn_i2c_of_match[] = {
    {
        .compatible = "brcm,rjn-i2c",
        .data = &rjn_i2c_quirks
    },
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

/* Driver Metadata */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramanujan SOC: Mirafra");
MODULE_DESCRIPTION("Platform: I2C Driver for RAMANUJAN");
