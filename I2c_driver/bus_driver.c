#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/of.h>

static int my_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    pr_info("My I2C device detected at address 0x%02x\n", client->addr);
    return 0;
}

static int my_i2c_remove(struct i2c_client *client)
{
    pr_info("My I2C device removed\n");
    return 0;
}

// I2C driver structure
static const struct i2c_device_id my_i2c_id[] = {
    { "my_i2c_device", 0 },
    { }
};

static struct i2c_driver my_i2c_driver = {
    .driver = {
        .name = "my_i2c_driver",
        .of_match_table = of_match_ptr(my_of_match),  // Optional: for device tree support
    },
    .probe = my_i2c_probe,
    .remove = my_i2c_remove,
    .id_table = my_i2c_id,
};

// Register the I2C driver
static int __init my_i2c_driver_init(void)
{
    return i2c_add_driver(&my_i2c_driver);
}

// Cleanup function
static void __exit my_i2c_driver_exit(void)
{
    i2c_del_driver(&my_i2c_driver);
}

module_init(my_i2c_driver_init);
module_exit(my_i2c_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("My I2C Bus Driver Example");
