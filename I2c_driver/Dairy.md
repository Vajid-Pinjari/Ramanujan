Date :17/01/25
/* Before meeting */
-> Started writing of platform driver.
-> Why we are taking our own i2c_structure.

/*After meeting*/
> spi and i2c speed comparison.
>clock speed in i2c.
> Started Writing probe functionality in platform driver.
> How compatible match with platform driver. (struct of_device_id).
> Get to know about how parameter passing will happpen to probe.
>i2c registers ( referred from tachnical ref. manual )
> struct completion completion;
> <platform_set_drvdata>
>devm_platform_get_and_ioremap_resource();


Date : 20/01/25
//Todays plan
> analyze and write that whole probe functionality & remove functionality.
>writel function
>i2c algo. functionalities.

/*Before meeting*/
> devm_clk_get //to get mclk (clock resource)
> mfr_i2c_register_div //to get bus_clk
> writing clock function.
   -> Writing mfr_i2c_register_dev();//clk devider function
   -> Written  struct clk_mfr_i2c
   -> initializes clk_ops
   -> written hw parametr function of clock
     * clk_mfr_i2c_set_rate,
     * clk_mfr_i2c_round_rate,
     * clk_mfr_i2c_recalc_rate
 -> container_of
