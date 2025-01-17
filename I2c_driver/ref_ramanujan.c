#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define MY_I2C_REG_CON   0x00  // Control register offset
#define MY_I2C_REG_STAT  0x04  // Status register offset
#define MY_I2C_REG_DATA  0x08  // Data register offset
#define MY_I2C_REG_CLK   0x0C  // Clock register offset

struct my_i2c_dev {
    void __iomem *base;      // Mapped I/O memory for the controller
    int irq;                 // Interrupt number
    struct i2c_adapter adap; // I2C adapter structure
    struct completion xfer_done; // Completion for transfer
};

static int my_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
    struct my_i2c_dev *dev = i2c_get_adapdata(adap);
    int i, ret = 0;

    for (i = 0; i < num; i++) {
        struct i2c_msg *msg = &msgs[i];

        // Set direction (read/write) in control register
        if (msg->flags & I2C_M_RD)
            iowrite32(0x01, dev->base + MY_I2C_REG_CON); // Set to read mode
        else
            iowrite32(0x00, dev->base + MY_I2C_REG_CON); // Set to write mode

        // Write or read data
        for (int j = 0; j < msg->len; j++) {
            if (msg->flags & I2C_M_RD) {
                // Wait for data availability
                if (!wait_for_completion_timeout(&dev->xfer_done, HZ)) {
                    ret = -ETIMEDOUT;
                    goto out;
                }

                msg->buf[j] = ioread32(dev->base + MY_I2C_REG_DATA);
            } else {
                iowrite32(msg->buf[j], dev->base + MY_I2C_REG_DATA);
                // Trigger transfer
                iowrite32(0x01, dev->base + MY_I2C_REG_STAT);

                // Wait for transfer complete
                if (!wait_for_completion_timeout(&dev->xfer_done, HZ)) {
                    ret = -ETIMEDOUT;
                    goto out;
                }
            }
        }
    }

out:
    return ret ? ret : num;
}

static u32 my_i2c_functionality(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static irqreturn_t my_i2c_irq_handler(int irq, void *dev_id)
{
    struct my_i2c_dev *dev = dev_id;

    // Clear interrupt in hardware
    iowrite32(0x0, dev->base + MY_I2C_REG_STAT);

    // Notify completion
    complete(&dev->xfer_done);

    return IRQ_HANDLED;
}

static const struct i2c_algorithm my_i2c_algo = {
    .master_xfer = my_i2c_master_xfer,
    .functionality = my_i2c_functionality,
};

static int my_i2c_probe(struct platform_device *pdev)
{
    struct resource *res;
    struct my_i2c_dev *dev;
    int ret;

    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    dev->base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(dev->base))
        return PTR_ERR(dev->base);

    dev->irq = platform_get_irq(pdev, 0);
    if (dev->irq < 0)
        return dev->irq;

    init_completion(&dev->xfer_done);

    ret = devm_request_irq(&pdev->dev, dev->irq, my_i2c_irq_handler, 0, dev_name(&pdev->dev), dev);
    if (ret)
        return ret;

    dev->adap.owner = THIS_MODULE;
    dev->adap.algo = &my_i2c_algo;
    dev->adap.dev.parent = &pdev->dev;
    snprintf(dev->adap.name, sizeof(dev->adap.name), "My I2C Bus");
    i2c_set_adapdata(&dev->adap, dev);

    ret = i2c_add_adapter(&dev->adap);
    if (ret)
        return ret;

    platform_set_drvdata(pdev, dev);

    dev_info(&pdev->dev, "My I2C controller driver initialized\n");
    return 0;
}

static int my_i2c_remove(struct platform_device *pdev)
{
    struct my_i2c_dev *dev = platform_get_drvdata(pdev);

    i2c_del_adapter(&dev->adap);
    return 0;
}

static const struct of_device_id my_i2c_of_match[] = {
    { .compatible = "mycompany,my-i2c", },
    { }
};
MODULE_DEVICE_TABLE(of, my_i2c_of_match);

static struct platform_driver my_i2c_driver = {
    .probe = my_i2c_probe,
    .remove = my_i2c_remove,
    .driver = {
        .name = "my_i2c_driver",
        .of_match_table = my_i2c_of_match,
    },
};

module_platform_driver(my_i2c_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("I2C Bus Driver for My SoC");
MODULE_LICENSE("GPL");
