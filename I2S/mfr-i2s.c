/*I2S Controller driver for Ramanujan soc*/

#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
//#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>

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
struct rjn_i2s_pins{
    u32 reg_offset;
    u32 shift;
};

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

static int rjn_snd_txctrl(struct rjn_i2s_dev *i2s,int on)
{
    unsigned int val = 0;
    int ret =0;

    spin_lock(&i2s->lock);
    if(on)
    {
        ret = regmap_update_bits(i2s->regmap,I2S_DMACR,
        0x0FFFFFFF,
        I2S_DMACR_TDE_ENABLE);
        if(ret<0)
            goto end;
        ret = regmap_update_bits(i2s->regmap, I2S_XFER,
					 I2S_XFER_TXS_START | I2S_XFER_RXS_START,
					 I2S_XFER_TXS_START | I2S_XFER_RXS_START);
        
        if (ret < 0)
			goto end;
		i2s->tx_start = true;
    }
    else{
		i2s->tx_start = false;
		ret = regmap_update_bits(i2s->regmap, I2S_DMACR,
					 I2S_DMACR_TDE_ENABLE,
					 I2S_DMACR_TDE_DISABLE);
		if (ret < 0)
			goto end;

		if (!i2s->rx_start) {
			ret = regmap_update_bits(i2s->regmap, I2S_XFER,
						 I2S_XFER_TXS_START | I2S_XFER_RXS_START,
						 I2S_XFER_TXS_STOP | I2S_XFER_RXS_STOP);
			if (ret < 0)
				goto end;
			udelay(150);
			ret = regmap_update_bits(i2s->regmap, I2S_CLR,
						 I2S_CLR_TXC | I2S_CLR_RXC,
						 I2S_CLR_TXC | I2S_CLR_RXC);
			if (ret < 0)
				goto end;
			ret = regmap_read_poll_timeout_atomic(i2s->regmap,
							      I2S_CLR,
							      val,
							      val == 0,
							      20,
							      200);
			if (ret < 0)
				dev_warn(i2s->dev, "fail to clear: %d\n", ret);
		}
    }

    end:
        spin_unlock(&i2s->lock);
	if (ret < 0)
		dev_err(i2s->dev, "lrclk update failed\n");

	return ret;
}


static int rjn_snd_rxctrl(struct rjn_i2s_dev *i2s, int on)
{
	unsigned int val = 0;
	int ret = 0;

	spin_lock(&i2s->lock);
	if (on) {
        /*
        RX_CONFIG

        1.Receiver enable ->(1<<0)
        2.receive channel->(1<<1)
        3.channel width -> (31<<6)
        4.data width -> (31<<12)
        5.clock mode -> (0<<26)(MASTER)
        6.clock polarity->
        */
		ret = regmap_update_bits(i2s->regmap, RX_CONFIG,
					 0x07FFFFFF,
					 (1<<RX_ENABLE)|(1<<RX_NUM_CHANNELS)|
                     (31<<RX_CHANNEL_WIDTH)|(31<<RX_DATA_WIDTH));
		if (ret < 0)
			goto end;

		ret = regmap_update_bits(i2s->regmap, I2S_XFER,
					 I2S_XFER_TXS_START | I2S_XFER_RXS_START,
					 I2S_XFER_TXS_START | I2S_XFER_RXS_START);
		if (ret < 0)
			goto end;
		i2s->rx_start = true;
	} 
    else {
		i2s->rx_start = false;

		ret = regmap_update_bits(i2s->regmap, I2S_DMACR,
					 I2S_DMACR_RDE_ENABLE,
					 I2S_DMACR_RDE_DISABLE);
		if (ret < 0)
			goto end;

		if (!i2s->tx_start) {
			ret = regmap_update_bits(i2s->regmap, I2S_XFER,
						 I2S_XFER_TXS_START | I2S_XFER_RXS_START,
						 I2S_XFER_TXS_STOP | I2S_XFER_RXS_STOP);
			if (ret < 0)
				goto end;
			udelay(150);
			ret = regmap_update_bits(i2s->regmap, I2S_CLR,
						 I2S_CLR_TXC | I2S_CLR_RXC,
						 I2S_CLR_TXC | I2S_CLR_RXC);
			if (ret < 0)
				goto end;
			ret = regmap_read_poll_timeout_atomic(i2s->regmap,
							      I2S_CLR,
							      val,
							      val == 0,
							      20,
							      200);
			if (ret < 0)
				dev_warn(i2s->dev, "fail to clear: %d\n", ret);
		}
	}
end:
	spin_unlock(&i2s->lock);
	if (ret < 0)
		dev_err(i2s->dev, "lrclk update failed\n");

	return ret;
}

//initialized DMA in dai probe 
static int rjn_i2s_dai_probe(struct snd_soc_dai *dai)
{
    //setting DMA data for playback and capture
    //1.take dai driver data from platform driver
    struct rjn_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);
    //set DMA Data
    snd_soc_dai_init_dma_data(dai,i2s->has_playback ? &i2s->playback_dma_data : NULL, 
                                i2s->has_capture ? &i2s->capture_dma_data : NULL);
    
    return 0;
}

static int rjn_i2s_dai_hw_params(struct snd_pcm_substream *substream ,struct snd_pcm_hw_params *params,struct 
                                snd_soc_dai *dai)
{
    struct rjn_i2s_dev *i2s = to_info(dai);
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	unsigned int val = 0;
	unsigned int mclk_rate, bclk_rate, div_bclk, div_lrck;

    //master mode clock cfg for bit clk and lrclk(ws) 
	if (i2s->is_master_mode) {
        //taking mclk_rate from platform drv
		mclk_rate = clk_get_rate(i2s->mclk);
		bclk_rate = i2s->bclk_ratio * params_rate(params);
		if (!bclk_rate)
			return -EINVAL;

        //divider for bitclock
		div_bclk = DIV_ROUND_CLOSEST(mclk_rate, bclk_rate);
        //divider for lrclk
		div_lrck = bclk_rate / params_rate(params);
        //update CKR 
		regmap_update_bits(i2s->regmap,TX_PRESCALER,
				   0xFFFFFFFF,(div_bclk<<16));
        regmap_update_bits(i2s->regmap,RX_PRESCALER,
				   0x7FFFFFFF,(div_bclk<<16));
		regmap_update_bits(i2s->regmap, I2S_CKR,
				   I2S_CKR_TSD_MASK |
				   I2S_CKR_RSD_MASK,
				   I2S_CKR_TSD(div_lrck) |
				   I2S_CKR_RSD(div_lrck));
	}

    //select i2s format
    switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		val |= I2S_TXCR_VDW(8);
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		val |= I2S_TXCR_VDW(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val |= I2S_TXCR_VDW(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val |= I2S_TXCR_VDW(24);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val |= I2S_TXCR_VDW(32);
		break;
	default:
		return -EINVAL;
	}

    //select channels for i2s
    switch (params_channels(params)) {
	case 8:
		val |= I2S_CHN_8;
		break;
	case 6:
		val |= I2S_CHN_6;
		break;
	case 4:
		val |= I2S_CHN_4;
		break;
	case 2:
		val |= I2S_CHN_2;
		break;

	default:
		dev_err(i2s->dev, "invalid channel: %d\n",
			params_channels(params));
		return -EINVAL;
	}

    //configure register for half duplex mode
    //Update RXCR register for capture
    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		regmap_update_bits(i2s->regmap, I2S_RXCR,
				   I2S_RXCR_VDW_MASK | I2S_RXCR_CSR_MASK,
				   val);
    //Update TXCR register for playback
    else
		regmap_update_bits(i2s->regmap, I2S_TXCR,
				   I2S_TXCR_VDW_MASK | I2S_TXCR_CSR_MASK,
				   val);


	// **REMOVED GRF CONFIGURATION HERE**
	// No need to update GRF for I²S direction control.

    //masking & updating TDL & RDL at 16 for TX & RX
    //(i.e at a time at least 16 words filled before start transfer)
    regmap_update_bits(i2s->regmap, I2S_DMACR, I2S_DMACR_TDL_MASK,
			   I2S_DMACR_TDL(16));
    //(i.e at least 16 words needed to fetch data)
	regmap_update_bits(i2s->regmap, I2S_DMACR, I2S_DMACR_RDL_MASK,
			   I2S_DMACR_RDL(16));

    //check which type of substream it is and cfg it.
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    val = I2S_CKR_TRCM_TXONLY;
    else
    val = I2S_CKR_TRCM_RXONLY;

    regmap_update_bits(i2s->regmap, I2S_CKR,
			   I2S_CKR_TRCM_MASK,
			   val);
    return 0;
}

int rjn_i2s_dai_bclk_ratio(struct snd_soc_dai *dai ,unsigned int ratio)
{
    struct rjn_i2s_dev *i2s = to_info(dai);
    //setting explicitly from user space
    //i.e override the setting which done in hw_params.
    i2s->bclk_ratio = ratio;

    return 0;
}

/*  
    why required set_clk?
    Your I2S controller or codec requires explicit configuration of the MCLK frequency.
    The driver structure requires informing the system about the MCLK source, even if it is fixed.
    Your audio codec needs MCLK frequency information for proper configuration.
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

int rjn_i2s_set_fmt(struct snd_soc_dai *dai, int fmt)
{
    struct rjn_i2s_dev *i2s = to_info(dai);
    int val=0,mask=0;
    int ret =0;

    mask = I2S_CKR_MSS_MASK;
    //check clk provided by
    switch(fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK)
    {
        case SND_SOC_DAIFMT_BC_FC:   //bit clk generated & frame clk generated
		/* Set source clock in Master mode */
		val = I2S_CKR_MSS_MASTER;
		i2s->is_master_mode = true;
		break;
	case SND_SOC_DAIFMT_BP_FP:  ////bit clk provided & frame clk provided
		val = I2S_CKR_MSS_SLAVE;
		i2s->is_master_mode = false;
		break;
	default:
		ret = -EINVAL;
        goto end;
    } 
    //update CKR register
    regmap_update_bits(i2s->regmap, I2S_CKR, mask, val);

    //set clock polarity
    //mask for clock polarity
    mask = I2S_CKR_CKP_MASK | I2S_CKR_TLP_MASK | I2S_CKR_RLP_MASK;
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:           //bitclk- Normal Frameclk-Normal
		val = I2S_CKR_CKP_NORMAL |       
		      I2S_CKR_TLP_NORMAL |
		      I2S_CKR_RLP_NORMAL;
		break;
	case SND_SOC_DAIFMT_NB_IF:           //bitclk- Normal Frameclk-Inverted
		val = I2S_CKR_CKP_NORMAL |
		      I2S_CKR_TLP_INVERTED |
		      I2S_CKR_RLP_INVERTED;
		break;
	case SND_SOC_DAIFMT_IB_NF:             //bitclk- Inverted Frameclk-Normal
		val = I2S_CKR_CKP_INVERTED |
		      I2S_CKR_TLP_NORMAL |
		      I2S_CKR_RLP_NORMAL;
		break;
	case SND_SOC_DAIFMT_IB_IF:             //bitclk- Inverted Frameclk-Inverted
		val = I2S_CKR_CKP_INVERTED |
		      I2S_CKR_TLP_INVERTED |
		      I2S_CKR_RLP_INVERTED;
		break;
	default:
		ret = -EINVAL;
		goto end;
	}
    regmap_update_bits(i2s->regmap,I2S_CKR,mask,val);

    //mode of data transfer
    mask = I2S_RXCR_IBM_MASK | I2S_RXCR_TFS_MASK | I2S_RXCR_PBM_MASK;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:             //right justified data format
		val = I2S_RXCR_IBM_RSJM;
		break;
	case SND_SOC_DAIFMT_LEFT_J:              //left justified data format
		val = I2S_RXCR_IBM_LSJM;
		break;
	case SND_SOC_DAIFMT_I2S:                //I2S normal data format
		val = I2S_RXCR_IBM_NORMAL;
		break;
	case SND_SOC_DAIFMT_DSP_A:              /* PCM delay 1 bit mode */
		val = I2S_RXCR_TFS_PCM | I2S_RXCR_PBM_MODE(1);
		break;
	case SND_SOC_DAIFMT_DSP_B:              /* PCM no delay mode */
		val = I2S_RXCR_TFS_PCM;
		break;
	default:
		ret = -EINVAL;
		goto end;
	}

    regmap_update_bits(i2s->regmap,I2S_CKR,mask,val);
end:
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
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
            if(substream->stream == SNDRV_PCM_STREAM_CAPTURE){
                if(!i2s->tx_start)
                clk_unprepare_disable(i2s->mclk);
                clk_unprepare_disable(i2s->bus_clk);
            ret = rjn_snd_rxctrl(i2s,0);
            }
            else{
                if(!i2s->rx_start)
                clk_disable_unprepare(i2s->bus_clk);
                clk_disable_unprepare(i2s->mclk);

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
    .ops = rjn_i2s_dai_ops ,
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
    { 0x00, 0xF3 },
    { 0x04, 0x0000000f},
    { 0x08, 0x00071f1f},
    { 0x10, 0x001f0000},
   // { 0x14, 0x00000000},
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
    .writeable_regs = rjn_i2s_writeable_regs,
    .readable_regs =  rjn_i2s_readable_regs,
    //.cache_type = /* */ ,

};

// Device Tree Match Table
static const struct of_device_id rjn_i2s_match[]
{
    { .compatible = "ramanujan,rjn-i2s", },
    { .compatible = "ramanujan,rjn-i2s0", .data = &mfr_i2s_pins },
    {},
};

/*Preparing DMA for playback and capture for i2s*/
static int rjn_i2s_init_dai(struct rjn_i2s_dev *i2s ,struct resource *res,struct sound_soc_dai_driver **dp)
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
        i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES; //same as for our SOC
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
    
    // initialize synchronization
    spin_lock_init(&i2s_dev->lock);
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

    //request irq
    irq = platform_get_irq(pdev,0);
    if(irq<0)
    {
        dev_err(&pdev->dev, "request irq error\n");
        return ret;
    }

    ret = devm_request_irq(&pdev->dev,irq,rjn_interrupt_handler,0,dev_name(&pdev->name),dev);
    if(ret<0)
    {
        dev_err(i2s_dev->dev,"failed to register divider",ret);
        return ret;
    }

    //bind base address to private structure
    i2s_dev->regs = regs;

    //initialize managed register map
    i2s_dev->regmap = devm_regmap_init_mmio(pdev->dev,regs,&rjn_i2s_regmap_config);
    if (IS_ERR(i2s_dev->regmap)) {
	    	dev_err(&pdev->dev,
		    	"Failed to initialise managed register map\n");
		    ret = PTR_ERR(i2s_dev->regmap);
		    goto err_clk;
	    }

    i2s_dev->dev = &pdev->dev;

    //select i2s as master mode
    i2s_dev->is_master_mode = true;

    //set private data structure with platform device structure
    platform_set_drvdata(pdev,i2s_dev);

    //initialize dai driver setup
    ret = rjn_i2s_init_dai(i2s_dev,res,&dai);
    if(ret)
    {
        goto err_clk;
    }

    //register component driver/DAI hardware (to inform ALSA)
    ret = devm_snd_soc_register_component(&pdev->dev,&rjn_i2s_cmpt_drv,&dai,0);
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

static const struct of_device_id rjn_i2s_match[] __maybe_unused = {
    { .compatible = "mirafra , rjn-i2s",},
    {},
};

MODULE_DEVICE_TABLE(of,rjn_i2s_match);

//Platform Driver Definition
static struct platform_driver mfr_i2s_driver = {
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

/* How to Enable Full Duplex Mode?

To enable full-duplex mode in an ALSA ASoC driver:
✔ Ensure separate TX and RX DMA channels are available.
✔ Do NOT set SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX, as this enforces half-duplex operation.
✔ In the trigger() function, enable both TX and RX DMA transfers simultaneously.
✔ Set appropriate audio format & clocking in set_fmt().
*/