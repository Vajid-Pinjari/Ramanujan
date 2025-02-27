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
    struct clk *hclk;
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
static const struct snd_soc_component_driver mfr_i2s_component = {
	,
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
    /* compatible strings for matching the device */
    {},
};


// Power Management (Optional but Important)

// Probe Function: Initializes the I2S Controller 
static int probe_platform(struct platform_device *pdev)
{


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
    .probe = probe_platform,
    .remove = mfr_i2s_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = of_match_ptr(mfr_i2s_match),
        /* .pm = &mfr_i2s_pm_ops, */
    },
};

module_platform_driver(mfr_i2s_driver);

MODULE_AUTHOR("Ramanujan : MFR I2S DRIVER");
MODULE_DESCRIPTION("An I2S Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform : " DRV_NAME);
