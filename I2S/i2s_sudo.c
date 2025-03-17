/*I2S Controller driver for Ramanujan soc*/

#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>

#include "i2s_headers.h" 

/* register mapping pending */

#define DRV_NAME "mfr-i2s"
/* 
 * Private Data Structure for I2S Controller
 * This structure holds device-specific information such as:
 * - Register base address
 * - Clock handles
 * - DMA configurations (if applicable)
 * - IRQ details
 * - Synchronization mechanisms
 */
struct mfr_i2s_pins{
    u32 reg_offset;
    u32 shift;
};

struct mfr_i2s_dev{
    struct device *dev;
    struct clk *bus_clk;
    struct clk *mclk;

    struct snd_dmaengine_dai_dma_data capture_dma_data;
    struct snd_dmaengine_dai_dma_data playback_dma_data;

    struct regmap *regmap;
    struct regmap *grf;

    bool has_capture;
    bool has_playback;

    bool tx_start;
    bool rx_start;
    bool is_master_mode;

    const struct mfr_i2s_pins *pins;
    unsigned int bclk_ratio;
    spinlock_t lock;

    struct pinctrl *pinctrl;
    struct pinctrl_state *bclk_on;
    struct pinctrl_state *bclk_off;    
    
};

/* 
 * DAI (Digital Audio Interface) Driver Definition 
 * This structure defines:
 * - Supported audio formats and rates
 * - Operations for configuring the I2S interface (e.g., hw_params, trigger)
 * - Playback & Capture capabilities
 */
static struct snd_soc_dai_driver rockchip_i2s_dai = {
	
};

/* 
 * ALSA SoC Component Driver
 * This represents the I2S controller as a component in the ALSA framework.
 * It defines:
 * - PCM handling functions (optional)
 * - DAPM (Dynamic Audio Power Management) support
 * - Codec interactions
 */

static const struct snd_soc_component_driver  mfr_i2s_cmpt_drv = {
    .name = DRV_NAME,
    .legacy_dai_naming = 0, //modern naming convention
};
/* 
 * Register Defaults
 * This structure stores default values for registers, 
 * used when initializing the hardware.
 */
static const struct reg_default mfr_i2s_reg_defaults[] = {
    { 0x00, 0x0000000f},
    { 0x04, 0x0000000f},
    { 0x08, 0x00071f1f},
    { 0x10, 0x001f0000},
   // { 0x14, 0x00000000},
};


static bool mfr_i2s_writeable_regs(struct device *dev,unsigned int reg)
{
    switch(reg)
    {
        case I2S_TXCR:
        case I2S_RXCR:
        case I2S_CKR:
        case I2S_DMACR:
        case I2S_INTCR:
        case I2S_XFER:
        case I2S_CLR:
        case I2S_TXDR:
            return true;
        default : 
            return false;
    }
    return true;
}


static bool mfr_i2s_readable_regs(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case I2S_TXCR:
	case I2S_RXCR:
	case I2S_CKR:
	case I2S_DMACR:
	case I2S_INTCR:
	case I2S_XFER:
	case I2S_CLR:
	case I2S_TXDR:
	case I2S_RXDR:
	case I2S_FIFOLR:
	case I2S_INTSR:
		return true;
	default:
		return false;
	}
    return true;
}

static bool mfr_i2s_volatile_regs(struct device *dev, unsigned int reg)
{
    switch(reg)
    {
        case I2S_INTSR:
        case I2S_CLR:
        case I2S_FIFOLR:
        case I2S_TXDR:
        case I2S_RXDR:
            return true;
        default:
            return false;
    }
    return true;
}

static bool mfr_i2s_precious_reg(struct device *dev,unsigned int reg)
{
    switch(reg)
    {
        case I2S_RXDR:
            return true;
        default:
            return false;
    }
    return true;
}
/* 
 * Regmap Configuration
 * This defines:
 * - Register bit width
 * - Valid address range
 * - Read/write access properties 
 */
static const struct regmap_config mfr_i2s_regmap_config = {
    .reg_bits = 32,
    .reg_stride = 4,  //
    .val_bits = 32,
    .max_register = I2S_RXDR,
    .num_reg_defaults = ARRAY_SIZE(mfr_i2s_regs_default),
    .writeable_regs = mfr_i2s_writeable_regs,
    .readable_regs =  mfr_i2s_readable_regs,
    .volatile_regs = mfr_i2s_volatile_regs,
    .precious_reg = mfr_i2s_precious_reg,
    //.cache_type = /* */ ,

};

// Device Tree Match Table
static const struct of_device_id mfr_i2s_match[]
{
    { .compatible = "ramanujan,mfr-i2s", },
    { .compatible = "ramanujan,mfr-i2s0", .data = &mfr_i2s_pins },
    {},
};

/*Preparing DMA for playback and capture for i2s*/
static int mfr_i2s_init_dai(struct mfr_i2s_dev *i2s ,struct resource *res,struct sound_soc_dai_driver **dp)
{
    struct device_node *node = i2s->dev->of_node;
    struct snd_soc_dai_driver *dai;
    struct property *dma_names;
    const char *dma_name;
    unsigned int val;


    //checking for dma in dts
    of_property_for_each_string(node,"dma-names",dma_names,dma_name)
    {
        if(!strcmp(dma_name, "tx"))
            i2s->has_playback = true;
        if(!strcmp(dma_names ,"rx"))
            i2s->has_capture = true;
    }

    dai = devm_kmemdup(i2s->dev, &mfr_i2s_dai, sizeof(*dai),GFP_KERNEL);
    if(!dai)
    {
        return -ENOMEM;
    }

    //prepare TX DMA for playback
    if(i2s->has_playback)
    {
        dai->playback.stream_name = "Playback";
        dai->playback.channels_min = 1;
        dai->playback.channels_max = 2;
        dai->playback.rates = SNDRV_PCM_RATE_8000_192000;
        dai->playback.formats = SNDRV_PCM_FMTBIT_S8	 |
                                SNDRV_PCM_FMTBIT_S16_LE |
                                SNDRV_PCM_FMTBIT_S20_3LE |
                                SNDRV_PCM_FMTBIT_S32_LE;

        i2s->playback_dma_data.addr = res->start + I2S_TXDR;
        i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        i2s->playback_dma_data.maxburst = 8;
    }

    //prepare RX DMA for capture
    if(i2s->has_capture)
    {
         dai->capture.stream_name = "Capture";
        dai->capture.channels_min = 1;
        dai->capture.channels_max = 2;
        dai->capture.rates = SNDRV_PCM_RATE_8000_192000;
        dai->capture.formats = SNDRV_PCM_FMTBIT_S8	 |
                                SNDRV_PCM_FMTBIT_S16_LE |
                                SNDRV_PCM_FMTBIT_S20_3LE |
                                SNDRV_PCM_FMTBIT_S32_LE |
                                SNDRV_PCM_FMTBIT_S32_LE;

        i2s->playback_dma_data.addr = res->start + I2S_RXDR;
        i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        i2s->playback_dma_data.maxburst = 8;

    }
    //link dai with platform dai
    if(dp)
    *dp = dai;
    //on sucess return zero
    return 0;
}

// Probe Function: Initializes the I2S Controller 
static int mfr_i2s_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct mfr_i2s_dev *i2s_dev;
    struct snd_soc_dai_driver *dai;
    struct resource *res;
    void __iomem *regs;
    int irq,ret;

    // Allocate memory to private data structure
    i2s_dev = devm_kzalloc(&pdev->dev , sizeof(*i2s_dev), GFP_KERNEL);
    if(!i2s_dev)
        return -ENOMEM;
    
    // initialize synchronization
    spin_lock_init(&i2s_dev->lock);
    i2s_dev->dev = &pdev->dev;

// Clock configuration
    //get bus_clk from Device tree
i2s_dev->bus_clk = devm_clk_get(&pdev->dev, "bus_clk");
if(IS_ERR(i2s_dev->bus_clk))
{
    dev_err(&pdev->dev , "Can't retreive i2s bus clock\n");
    return PTR_ERR(i2s_dev->bus_clk);
}
//prepare and enable bus_clk
ret = clk_prepare_enable(i2s_dev->bus_clk);
if(ret)
{
    dev_err(i2s_dev->dev, "hclock enable failed %d\n",ret);
    return ret;
}

//get mclk from Device tree
i2s_dev->mclk = devm_clk_get(&pdev->dev, "I2S_Master_C");
if(IS_ERR(i2s_dev->mclk))
{
    dev_err(&pdev->dev , "Can't retreive i2s bus clock\n");
    return PTR_ERR(i2s_dev->mclk);
}
//prepare and enable mclk
ret = clk_prepare_enable(i2s_dev->mclk);
if(ret)
{
    dev_err(i2s_dev->dev, "mclk enable failed %d\n",ret);
    return ret;
}
//get base address from pdev->resource
regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
if (IS_ERR(regs)) {
ret = PTR_ERR(regs);
	goto err_clk;
}

//initialize managed register map
i2s_dev->regmap = devm_regmap_init_mmio(pdev->dev,regs,&mfr_i2s_regmap_config);
if (IS_ERR(i2s_dev->regmap)) {
		dev_err(&pdev->dev,
			"Failed to initialise managed register map\n");
		ret = PTR_ERR(i2s_dev->regmap);
		goto err_clk;
	}

i2s_dev->dev = &pdev->dev;

//set private data structure with platform device structure
platform_set_drvdata(pdev,i2s_dev);

//initialize dai driver setup
ret = mfr_i2s_init_dai(i2s_dev,res,&dai);
if(ret)
{
    goto err_clk;
}

//register component driver/DAI hardware (to inform ALSA)
ret = devm_snd_soc_register_component(&pdev->dev,&mfr_i2s_cmpt_drv,&dai,0);
if(ret)
{
    dev_err(&pdev->dev, "Could not register DAI \n");
		goto err_suspend;
}

//register pcm for default dma for user level app
ret = devm_snd_dmaengine_pcm_register(&pdev->dev,NULL,0);
if(ret)
{
    dev_err(&pdev->dev, "Could not register PCM\n");
    goto err_suspend;
}

err_suspend:
   // i2s_runtime_suspend(&pdev->dev);
err_clk:
    clk_disable_unprepare(i2s_dev->bus_clk);
    clk_disable_unprepare(i2s_dev->mclk);
}

//* Remove Function: Cleans up resources
static void remove_platform()
{
    
}

static const struct of_device_id mfr_i2s_match[] __maybe_unused = {
    { .compatible = "mirafra , mfr-i2s",},
    {},
};

MODULE_DEVICE_TABLE(of,mfr_i2s_match);

//Platform Driver Definition
static struct platform_driver mfr_i2s_driver = {
    .probe = mfr_i2s_probe,
    .remove = mfr_i2s_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = of_match_ptr(mfr_i2s_match),
        /* .pm = power gating is off */
    },
};

module_platform_driver(mfr_i2s_driver);

MODULE_AUTHOR("Ramanujan : MFR I2S DRIVER");
MODULE_DESCRIPTION("An I2S Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform : " DRV_NAME);
