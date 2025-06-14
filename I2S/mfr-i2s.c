/*I2S Controller driver for Ramanujan soc*/

#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/types.h>
#include <linux/stddef.h>  
//#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include "i2s_headers.h" 

/* register mapping pending */

#define DRV_NAME "rjn-i2s"
/* 
 * Private Data Structure for I2S Controller
 * This structure holds device-specific information such as:
 * - Register base address
 * - Clock handles
 * - DMA configurations (if applicable)
 * - IRQ details
 * - Synchronization mechanisms
 */

struct rjn_i2s_dev{
    struct device *dev;
    struct clk *bus_clk;
    struct clk *mclk;

    struct snd_dmaengine_dai_dma_data capture_dma_data;
    struct snd_dmaengine_dai_dma_data playback_dma_data;

    struct regmap *regmap;
    //struct regmap *grf;

    void __iomem *regs;
    
    bool has_capture;
    bool has_playback;

    bool tx_start;
    bool rx_start;
    bool is_master_mode;


/*
 * Used to indicate the tx/rx status.
 * I2S controller hopes to start the tx and rx together,
 * also to stop them when they are both try to stop.
*/
    const struct rjn_i2s_pins *pins;
    unsigned int bclk_ratio;
    spinlock_t lock;

    struct pinctrl *pinctrl;
    struct pinctrl_state *bclk_on;
    struct pinctrl_state *bclk_off;    
    
};


static inline struct rjn_i2s_dev *to_info(struct snd_soc_dai *dai)
{
	return snd_soc_dai_get_drvdata(dai);
}

static irqreturn_t rjn_interrupt_handler(int irq, void *dev_id)
{
    struct rjn_i2s_dev *i2s = dev_id;
    u32 irq_status;

    // Read the IRQ status register
    irq_status = readl(i2s->regs + IRQ_STATUS);

    // Check and log specific interrupt events
    if (irq_status & IRQ_RX_READY_MASK)
        dev_info(i2s->dev, "RX Ready interrupt occurred\n");

    if (irq_status & IRQ_TX_FIFO_EMPTY_MASK)
        dev_info(i2s->dev, "TX FIFO Empty interrupt occurred\n");

    if (irq_status & IRQ_TX_FIFO_UR_MASK)
        dev_info(i2s->dev, "TX FIFO Underrun interrupt occurred\n");

    if (irq_status & IRQ_RX_FIFO_OR_MASK)
        dev_info(i2s->dev, "RX FIFO Overrun interrupt occurred\n");

    // Clear only the handled interrupts by writing back the status
    writel(irq_status, i2s->regs + IRQ_STATUS);

    return IRQ_HANDLED;
}


static int rjn_snd_txctrl(struct rjn_i2s_dev *i2s, int enable)
{
    u32 val;
    int ret;

    ret = regmap_read(i2s->regmap, TX_CONFIG, &val);
    if (ret)
        return ret;

    if (enable) {
        val |= TX_ENABLE_MASK;
        i2s->tx_start = true;

        // Enable TX FIFO interrupts (minimal: Empty + Underrun)
        ret = regmap_update_bits(i2s->regmap, IRQ_ENABLE,
                                 IRQ_TX_FIFO_EMPTY_MASK | IRQ_TX_FIFO_UR_MASK,
                                 IRQ_TX_FIFO_EMPTY_MASK | IRQ_TX_FIFO_UR_MASK);
        if (ret)
            return ret;
    } else {
        val &= ~TX_ENABLE_MASK;
        i2s->tx_start = false;

        // Disable TX interrupts
        ret = regmap_update_bits(i2s->regmap, IRQ_ENABLE,
                                 IRQ_TX_FIFO_EMPTY_MASK | IRQ_TX_FIFO_UR_MASK,
                                 0);
        if (ret)
            return ret;
    }

    return regmap_write(i2s->regmap, TX_CONFIG, val);
}



static int rjn_snd_rxctrl(struct rjn_i2s_dev *i2s, int enable)
{
    u32 val;
    int ret;

    ret = regmap_read(i2s->regmap, RX_CONFIG, &val);
    if (ret)
        return ret;

    if (enable) {
        val |= RX_ENABLE_MASK;
        i2s->rx_start = true;

        // Enable RX FIFO interrupts (minimal: Data Ready + Overrun)
        ret = regmap_update_bits(i2s->regmap, IRQ_ENABLE,
                                 IRQ_RX_READY_MASK | IRQ_RX_FIFO_OR_MASK,
                                 IRQ_RX_READY_MASK | IRQ_RX_FIFO_OR_MASK);
        if (ret)
            return ret;
    } else {
        val &= ~RX_ENABLE_MASK;
        i2s->rx_start = false;

        // Disable RX interrupts
        ret = regmap_update_bits(i2s->regmap, IRQ_ENABLE,
                                 IRQ_RX_READY_MASK | IRQ_RX_FIFO_OR_MASK,
                                 0);
        if (ret)
            return ret;
    }

    return regmap_write(i2s->regmap, RX_CONFIG, val);
}

//initialized DMA in dai probe 
static int rjn_i2s_dai_probe(struct snd_soc_dai *dai)
{
    //setting addresses for playback and capture
    //1.take dai driver data from platform driver
    struct rjn_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);
    //set DMA Data
    snd_soc_dai_init_dma_data(dai,i2s->has_playback ? &i2s->playback_dma_data : NULL, 
                                i2s->has_capture ? &i2s->capture_dma_data : NULL);
    
    return 0;
}

static int rjn_i2s_dai_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params,
                                 struct snd_soc_dai *dai)
{
    struct rjn_i2s_dev *i2s = to_info(dai);
    unsigned int val_tx = 0, val_rx = 0;
    unsigned int mclk_rate, bclk_rate, div_bclk;

    if (i2s->is_master_mode) {
        mclk_rate = clk_get_rate(i2s->mclk);
        bclk_rate = i2s->bclk_ratio * params_rate(params);
        if (!bclk_rate)
            return -EINVAL;

        div_bclk = DIV_ROUND_CLOSEST(mclk_rate, bclk_rate);

        regmap_update_bits(i2s->regmap, TX_PRESCALER,
                           TX_MCLK_DIV_MASK | TX_I2S_DIV_MASK,
                           (1 << TX_MCLK_DIV) | (div_bclk << TX_I2S_DIV));

        regmap_update_bits(i2s->regmap, RX_PRESCALER,
                           RX_MCLK_DIV_MASK | RX_I2S_DIV_MASK,
                           (1 << RX_MCLK_DIV) | (div_bclk << RX_I2S_DIV));
    }

    // Format (data width)
    switch (params_format(params)) {
    case SNDRV_PCM_FORMAT_S8:
        val_tx |= (8 << TX_DATA_WIDTH);
        val_rx |= (8 << RX_DATA_WIDTH);
        break;
    case SNDRV_PCM_FORMAT_S16_LE:
        val_tx |= (16 << TX_DATA_WIDTH);
        val_rx |= (16 << RX_DATA_WIDTH);
        break;
    case SNDRV_PCM_FORMAT_S20_3LE:
        val_tx |= (20 << TX_DATA_WIDTH);
        val_rx |= (20 << RX_DATA_WIDTH);
        break;
    case SNDRV_PCM_FORMAT_S24_LE:
        val_tx |= (24 << TX_DATA_WIDTH);
        val_rx |= (24 << RX_DATA_WIDTH);
        break;
    case SNDRV_PCM_FORMAT_S32_LE:
        val_tx |= (32 << TX_DATA_WIDTH);
        val_rx |= (32 << RX_DATA_WIDTH);
        break;
    default:
        return -EINVAL;
    }

    //No of Channels
    switch (params_channels(params)) {
    case 8:
        val_tx |= (8 << TX_NUM_CHANNELS);
        val_rx |= (8 << RX_NUM_CHANNELS);
        break;
    case 6:
        val_tx |= (6 << TX_NUM_CHANNELS);
        val_rx |= (6 << RX_NUM_CHANNELS);
        break;
    case 4:
        val_tx |= (4 << TX_NUM_CHANNELS);
        val_rx |= (4 << RX_NUM_CHANNELS);
        break;
    case 2:
        val_tx |= (2 << TX_NUM_CHANNELS);
        val_rx |= (2 << RX_NUM_CHANNELS);
        break;
    default:
        dev_err(i2s->dev, "Invalid channel count: %d\n", params_channels(params));
        return -EINVAL;
    }

    // Common fields 
    val_tx |= (1 << TX_ENABLE) | (31 << TX_CHANNEL_WIDTH) |
              (0 << TX_PROTO_MODE) | (0 << TX_BIT_ORDER);

    val_rx |= (1 << RX_ENABLE) | (31 << RX_CHANNEL_WIDTH) |
              (0 << RX_PROTO_MODE) | (0 << RX_BIT_ORDER);

    // Final register write
    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
        regmap_update_bits(i2s->regmap, RX_CONFIG,
            RX_NUM_CHANNELS_MASK | RX_DATA_WIDTH_MASK |
            RX_CHANNEL_WIDTH_MASK | RX_PROTO_MODE_MASK |
            RX_BIT_ORDER_MASK | RX_ENABLE_MASK,
            val_rx);
    }
    else {
        regmap_update_bits(i2s->regmap, TX_CONFIG,
            TX_NUM_CHANNELS_MASK | TX_DATA_WIDTH_MASK |
            TX_CHANNEL_WIDTH_MASK | TX_PROTO_MODE_MASK |
            TX_BIT_ORDER_MASK | TX_ENABLE_MASK,
            val_tx);
    }

    return 0;
}

//defines how many bit clocks (BCLK cycles) occur per frame	
int rjn_i2s_dai_bclk_ratio(struct snd_soc_dai *dai ,unsigned int ratio)
{
    struct rjn_i2s_dev *i2s = to_info(dai);

    i2s->bclk_ratio = ratio;

    return 0;
}

/*  
    why required set_sysclk?
    Your I2S controller or codec requires explicit configuration of the MCLK frequency.
    The driver structure requires informing the system about the MCLK source, even if it is fixed.
    Your audio codec needs MCLK frequency information for proper configuration.
    i.e defines how many bits are transferred per audio word
*/
int rjn_i2s_set_sysclk(struct snd_soc_dai *dai,int clk_id,unsigned int freq,int dir)
{
    int ret;
    struct rjn_i2s_dev *i2s = to_info(dai);

    if(freq==0)
    return 0;
    //set clk rate of given freq from app layer
    ret = clk_set_rate(i2s->mclk,freq);
    if (ret)
		dev_err(i2s->dev, "Fail to set mclk %d\n", ret);

	return ret;
}

int rjn_i2s_set_fmt(struct snd_soc_dai *dai,unsigned int fmt)
{
    struct rjn_i2s_dev *i2s = to_info(dai);
    int ret = 0;
    unsigned int val_tx = 0, val_rx = 0;

    /* ---------- Clock Mode: Master or Slave ---------- */
            /*P-Provider C-consumer*/
    switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
    case SND_SOC_DAIFMT_BP_FP:  // Master
        val_tx = (0 << TX_CLK_MODE);
        val_rx = (0 << RX_CLK_MODE);
        i2s->is_master_mode = true;
        break;

    case SND_SOC_DAIFMT_BC_FC:  // Slave
        val_tx = (1 << TX_CLK_MODE);
        val_rx = (1 << RX_CLK_MODE);
        i2s->is_master_mode = false;
        break;

    default:
        return -EINVAL;
    }

    regmap_update_bits(i2s->regmap, TX_CONFIG,
                       TX_CLK_MODE_MASK, val_tx);
    regmap_update_bits(i2s->regmap, RX_CONFIG,
                       RX_CLK_MODE_MASK, val_rx);

    /* ---------- Clock Polarity ---------- */
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
    case SND_SOC_DAIFMT_NB_NF:  // Normal BCLK
        val_tx = (0 << TX_CLK_POL);
        val_rx = (0 << RX_CLK_POL);
        break;

    case SND_SOC_DAIFMT_IB_IF:  // Inverted BCLK + Inverted LRCLK
        val_tx = (1 << TX_CLK_POL);
        val_rx = (1 << RX_CLK_POL);
        dev_warn(i2s->dev, "Frame clock (LRCLK) inversion not supported, ignoring\n");
        break;

    default:
        return -EINVAL;
    }

    regmap_update_bits(i2s->regmap, TX_CONFIG,
                       TX_CLK_POL_MASK, val_tx);
    regmap_update_bits(i2s->regmap, RX_CONFIG,
                       RX_CLK_POL_MASK, val_rx);

    /* ---------- Justification & WS Delay ---------- */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_RIGHT_J:
        val_tx = (1 << TX_JUSTIFICATION) | (0 << TX_SD_DELAY);
        val_rx = (1 << RX_JUSTIFICATION) | (0 << RX_SD_DELAY);
        break;

    case SND_SOC_DAIFMT_LEFT_J:
        val_tx = (0 << TX_JUSTIFICATION) | (0 << TX_SD_DELAY);
        val_rx = (0 << RX_JUSTIFICATION) | (0 << RX_SD_DELAY);
        break;

    default:  // Assume I2S (Philips standard)
        val_tx = (0 << TX_JUSTIFICATION) | (1 << TX_SD_DELAY);
        val_rx = (0 << RX_JUSTIFICATION) | (1 << RX_SD_DELAY);
        break;
    }

    regmap_update_bits(i2s->regmap, TX_CONFIG,
                       TX_JUSTIFICATION_MASK | TX_SD_DELAY_MASK,
                       val_tx);
    regmap_update_bits(i2s->regmap, RX_CONFIG,
                       RX_JUSTIFICATION_MASK | RX_SD_DELAY_MASK,
                       val_rx);

    return ret;
}


//responsible for communication
static int rjn_i2s_trigger(struct snd_pcm_substream *substream,int cmd,struct snd_soc_dai *dai)
{
    struct rjn_i2s_dev *i2s = to_info(dai);
    int ret =0;
    
    switch(cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
            if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
                ret = rjn_snd_rxctrl(i2s,1);
            else 
                ret = rjn_snd_txctrl(i2s,1);
            if(ret<0)
                return ret;
                clk_prepare_enable(i2s->mclk);
                clk_prepare_enable(i2s->bus_clk);
            break;
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
            if(substream->stream == SNDRV_PCM_STREAM_CAPTURE){
                if(!i2s->tx_start)
                {
                clk_disable_unprepare(i2s->mclk);
                clk_disable_unprepare(i2s->bus_clk);
                }
            ret = rjn_snd_rxctrl(i2s,0);
            }
            else{
                if(!i2s->rx_start)
                {
                clk_disable_unprepare(i2s->mclk);
                clk_disable_unprepare(i2s->bus_clk);
                }
                ret = rjn_snd_txctrl(i2s,0);
            }
            break;
        default:
            ret = -EINVAL;
            break;
    }
    return ret;
}

static const struct snd_soc_dai_ops rjn_i2s_dai_ops = {
    .probe = rjn_i2s_dai_probe,
    .hw_params = rjn_i2s_dai_hw_params,
    .set_bclk_ratio = rjn_i2s_dai_bclk_ratio,
    .set_sysclk = rjn_i2s_set_sysclk,
    .set_fmt = rjn_i2s_set_fmt,
    .trigger = rjn_i2s_trigger,
};
/* 
 * DAI (Digital Audio Interface) Driver Definition 
 * This structure defines:
 * - Supported audio formats and rates
 * - Operations for configuring the I2S interface (e.g., hw_params, trigger)
 * - Playback & Capture capabilities
 */
static struct snd_soc_dai_driver rjn_i2s_dai = {
    .ops = &rjn_i2s_dai_ops,
    .symmetric_rate = 1,
};

/* 
 * ALSA SoC Component Driver
 * This represents the I2S controller as a component in the ALSA framework.
 * It defines:
 * - PCM handling functions (optional)
 * - DAPM (Dynamic Audio Power Management) support
 * - Codec interactions
 */

static const struct snd_soc_component_driver  rjn_i2s_cmpt_drv = {
    .name = DRV_NAME,
    .legacy_dai_naming = 0, //modern naming convention
};
/* 
 * Register Defaults
 * This structure stores default values for registers, 
 * used when initializing the hardware.
 */
static const struct reg_default rjn_i2s_reg_defaults[] = {
    { 0x00, 0xF3C4 },
    { 0x04, 0xF3C4},
    { 0x08, 0x2F0 },
    { 0x0c, 0x2F0},
};


static bool rjn_i2s_writeable_regs(struct device *dev,unsigned int reg)
{
    switch(reg)
    {
        case TX_CONFIG:                  
        case RX_CONFIG :                    
        case TX_PRESCALER :                
        case RX_PRESCALER :
        case IRQ_ENABLE :
        case IRQ_STATUS :
        case FIFO_FLUSH :  
        case TX_FIFO :
         return true;
        default : 
            return false;
    }
    return true;
}


static bool rjn_i2s_readable_regs(struct device *dev, unsigned int reg)
{
	switch (reg) {
	    case TX_CONFIG:                  
        case RX_CONFIG :                    
        case TX_PRESCALER :                
        case RX_PRESCALER :
        case IRQ_ENABLE :
        case IRQ_STATUS :
        case FIFO_STATUS:  
        case RX_FIFO :
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
static const struct regmap_config rjn_i2s_regmap_config = {
    .reg_bits = 32,
    .reg_stride = 4,  //
    .val_bits = 32,
    .max_register = SOC_TIMEOUT,
    .writeable_reg = rjn_i2s_writeable_regs,
    .readable_reg =  rjn_i2s_readable_regs,
    //.cache_type = /* */ ,

};

// Device Tree Match Table
static const struct of_device_id rjn_i2s_match[] =
{
    { .compatible = "ramanujan,rjn-i2s", },
    { .compatible = "ramanujan,rjn-i2s0", },
    {},
};

/*Preparing DMA for playback and capture for i2s*/
static int rjn_i2s_init_dai(struct rjn_i2s_dev *i2s ,struct resource *res,struct snd_soc_dai_driver **dp)
{
    struct device_node *node = i2s->dev->of_node;
    struct snd_soc_dai_driver *dai;
    struct property *dma_names;
    const char *dma_name;
    unsigned int val;

    //checking for dma in dts
    /*
    of_property_for_each_string(node,"dma-names",dma_names,dma_name)
    {
        if(!strcmp(dma_name, "tx"))
            i2s->has_playback = true;
        if(!strcmp(dma_name ,"rx"))
            i2s->has_capture = true;
    }
    */
    i2s->has_playback = true;
    i2s->has_capture = true;

    dai = devm_kmemdup(i2s->dev, &rjn_i2s_dai, sizeof(*dai),GFP_KERNEL);
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

        i2s->playback_dma_data.addr = res->start + TX_FIFO;
       // i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES; //same as for our SOC
        i2s->playback_dma_data.maxburst = 8;  //at a time how many words can read or write
    }

    //prepare RX stream (for ALSA pcm layer not actual DMA) for capture
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

        i2s->playback_dma_data.addr = res->start + RX_FIFO;
        //i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES; //only for DMA
        i2s->playback_dma_data.maxburst = 8;

    }
    //link dai with platform dai
    if(dp)
    *dp = dai;
    //on sucess return zero
    return 0;
}

static void rjn_i2s_init(rjn_i2s_dev *i2s_dev)
{
    /********Enable TX and RX in I2S**********/
    regmap_update_bits(i2s_dev->regmap,TX_CONFIG,TX_ENABLE_MASK,TX_ENABLE_MASK);
    regmap_update_bits(i2s_dev->regmap,RX_CONFIG,RX_ENABLE_MASK,RX_ENABLE_MASK);

    /********set threshould value for FIFO*********/
    regmap_update_bits(i2s_dev->regmap,FIFO_THRESHOLD,
                        RX_FIFO_THRESHOLD_MASK|TX_FIFO_THRESHOLD_MASK,
                        64<<RX_FIFO_THRESHOLD|64<<TX_FIFO_THRESHOLD);

    /********set SOC_TIMEOUT value for TIMEOUT register***********/
    regmap_write(i2s_dev->regmap,SOC_TIMEOUT,64);
}

// Probe Function: Initializes the I2S Controller 
static int rjn_i2s_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct rjn_i2s_dev *i2s_dev;
    struct snd_soc_dai_driver *dai;
    struct resource *res;
    void __iomem *regs;
    int irq,ret;

    // Allocate memory to private data structure
    i2s_dev = devm_kzalloc(&pdev->dev , sizeof(*i2s_dev), GFP_KERNEL);
    if(!i2s_dev)
        return -ENOMEM;
    
    i2s_dev->dev = &pdev->dev;

    //get base address from pdev->resource
    regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
    if (IS_ERR(regs)) {
    ret = PTR_ERR(regs);
    	goto err_clk;
    }

    // Clock configuration
    //get bus_clk from Device tree
    i2s_dev->bus_clk = devm_clk_get(&pdev->dev, "bus_clk");
    if(IS_ERR(i2s_dev->bus_clk))
    {
        dev_err(&pdev->dev , "Can't retreive i2s bus clock\n");
        return PTR_ERR(i2s_dev->bus_clk);
    }
    //prepare and enable bus_clk is not required because it already enabled by apb

    //get mclk from Device tree
    i2s_dev->mclk = devm_clk_get(&pdev->dev, "I2S_Master_C");
    if(IS_ERR(i2s_dev->mclk))
    {
        dev_err(&pdev->dev , "Can't retreive i2s master clock\n");
        return PTR_ERR(i2s_dev->mclk);
    }
    //prepare and enable mclk
    ret = clk_prepare_enable(i2s_dev->mclk);
    if(ret)
    {
        dev_err(i2s_dev->dev, "mclk enable failed %d\n",ret);
        return ret;
    }

    //request irq
    irq = platform_get_irq(pdev,0);
    if(irq<0)
    {
        dev_err(&pdev->dev, "request irq error\n");
        return ret;
    }

    ret = devm_request_irq(&pdev->dev, irq, rjn_interrupt_handler, 0, dev_name(&pdev->dev), &pdev->dev);

    if(ret<0)
    {
        dev_err(i2s_dev->dev,"failed to register divider");
        return ret;
    }

    //bind base address to private structure
    i2s_dev->regs = regs;

    //initialize managed register map
    i2s_dev->regmap = devm_regmap_init_mmio(&pdev->dev,regs,&rjn_i2s_regmap_config);
    if (IS_ERR(i2s_dev->regmap)) {
	    	dev_err(&pdev->dev,
		    	"Failed to initialise managed register map\n");
		    ret = PTR_ERR(i2s_dev->regmap);
		    goto err_clk;
	    }

    i2s_dev->dev = &pdev->dev;

    //set private data structure with platform device structure
    platform_set_drvdata(pdev,i2s_dev);

    //init i2s
    rjn_init_i2s(i2s_dev);
    //initialize dai driver setup
    ret = rjn_i2s_init_dai(i2s_dev,res,&dai);
    if(ret)
    {
        goto err_clk;
    }

    //register component driver/DAI hardware (to inform ALSA)
    ret = devm_snd_soc_register_component(&pdev->dev,&rjn_i2s_cmpt_drv,dai,0);
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
    return 0;
}

//* Remove Function: Cleans up resources
static void rjn_i2s_remove(struct platform_device *pdev)
{
    struct rjn_i2s_dev *i2s_dev = platform_get_drvdata(pdev);
    clk_disable_unprepare(i2s_dev->bus_clk);
    clk_disable_unprepare(i2s_dev->mclk);
   // return 0;
}

MODULE_DEVICE_TABLE(of,rjn_i2s_match);

//Platform Driver Definition
static struct platform_driver rjn_i2s_driver = {
    .probe = rjn_i2s_probe,
    .remove = rjn_i2s_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = of_match_ptr(rjn_i2s_match),
        /* .pm = power gating is off */
    },
};

module_platform_driver(rjn_i2s_driver);

MODULE_AUTHOR("Ramanujan : RJN I2S DRIVER");
MODULE_DESCRIPTION("An I2S Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform : " DRV_NAME);
