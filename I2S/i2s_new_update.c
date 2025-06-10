/*I2S Controller driver for Ramanujan SoC*/

#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/types.h>
#include <linux/stddef.h>  
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <sound/asoc.h>

/*I2S core register data from SmartDV Datasheet*/
#include "i2s_headers.h" 

/*Function prototypes*/
static int rjn_i2s_dai_hw_params(struct snd_pcm_substream *substream,
                          struct snd_pcm_hw_params *params,
                          struct snd_soc_dai *dai);
static int rjn_i2s_dai_bclk_ratio(struct snd_soc_dai *dai, unsigned int ratio);
static int rjn_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id,
                       unsigned int freq, int dir);
static int rjn_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt);
static int rjn_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
                    struct snd_soc_dai *dai);

#define DRV_NAME "rjn-i2s"

/* Private Data Structure for I2S Controller */
struct rjn_i2s_dev {
    struct device *dev;           // Pointer to device structure
    struct clk *mclk;             // Master clock for I2S
    struct regmap *regmap;        // Register map for I2S registers
    void __iomem *regs;           // I/O memory for direct register access
    bool tx_start;                // Flag indicating TX (playback) is active
    bool rx_start;                // Flag indicating RX (capture) is active
    void *tx_buf;                 // Pointer to TX PCM buffer (interleaved samples)
    size_t tx_len;                // Number of TX frames to process
    size_t tx_pos;                // Current TX frame index
    void *rx_buf;                 // Pointer to RX PCM buffer (interleaved samples)
    size_t rx_len;                // Number of RX frames to process
    size_t rx_pos;                // Current RX frame index
    int irq;                      // IRQ number from DTS or platform
    spinlock_t lock;              // Spinlock (retained for future use)
    bool is_master_mode;          // Flag indicating master mode operation
    unsigned int bclk_ratio;      // Bit clock ratio for master mode
    unsigned int channels;        // Number of audio channels (e.g., 2 for stereo)
    snd_pcm_format_t format;      // PCM format (e.g., S16_LE, S32_LE)
};

/* Utility function to retrieve driver data from DAI */
static inline struct rjn_i2s_dev *to_info(struct snd_soc_dai *dai)
{
    return snd_soc_dai_get_drvdata(dai);
}

/* Fill TX FIFO with PCM data for playback */
static void rjn_i2s_fill_txfifo(struct rjn_i2s_dev *i2s)
{
    // Continue until all frames are processed or FIFO is full
    while (i2s->tx_pos < i2s->tx_len) {
        __u32 fifo_status; // FIFO status register value
        // Read IRQ status to check FIFO state
        regmap_read(i2s->regmap, IRQ_STATUS, &fifo_status);

        // Stop if TX FIFO is full
        if (fifo_status & IRQ_TX_FIFO_FULL)
            break;

        // Write one frame to TX FIFO (assumes 32-bit FIFO writes)
        // Extract sample based on PCM format and pad to 32-bit
        __u32 sample = 0;
        switch (i2s->format) {
        case SNDRV_PCM_FORMAT_S8:
            sample = ((int8_t *)i2s->tx_buf)[i2s->tx_pos] << 24; // Sign-extend 8-bit
            break;
        case SNDRV_PCM_FORMAT_S16_LE:
            sample = ((int16_t *)i2s->tx_buf)[i2s->tx_pos] << 16; // Sign-extend 16-bit
            break;
        case SNDRV_PCM_FORMAT_S24_LE:
            sample = ((int32_t *)i2s->tx_buf)[i2s->tx_pos] & 0xFFFFFF00; // Mask to 24-bit
            break;
        case SNDRV_PCM_FORMAT_S32_LE:
            sample = ((uint32_t *)i2s->tx_buf)[i2s->tx_pos]; // Full 32-bit
            break;
        default:
            dev_err(i2s->dev, "Unsupported TX format: %d\n", i2s->format);
            return;
        }
        regmap_write(i2s->regmap, TX_FIFO, sample);
        i2s->tx_pos++; // Advance to next frame

        // Log progress for debugging
        dev_dbg(i2s->dev, "TX: pos=%zu, len=%zu, format=%d\n", i2s->tx_pos, i2s->tx_len, i2s->format);
    }

    // Check for buffer overrun (debug)
    if (i2s->tx_pos > i2s->tx_len) {
        dev_err(i2s->dev, "TX overrun: pos=%zu, len=%zu\n", i2s->tx_pos, i2s->tx_len);
    }

    // Check if all frames are processed
    if (i2s->tx_pos >= i2s->tx_len) {
        i2s->tx_start = false; // Mark TX as complete
        // Disable only TX interrupts that were raised
        regmap_update_bits(i2s->regmap, IRQ_ENABLE, IRQ_TX_FIFO_EMPTY | IRQ_TX_FIFO_TT, 0);
        // Disable TX block
        regmap_update_bits(i2s->regmap, TX_CONFIG, TX_ENABLE_MASK, 0);
    }
}

/* Drain RX FIFO into PCM buffer for capture */
static void rjn_i2s_drain_rxfifo(struct rjn_i2s_dev *i2s)
{
    // Continue until all frames are processed or FIFO is empty
    while (i2s->rx_pos < i2s->rx_len) {
        __u32 fifo_status; // FIFO status register value
        // Read IRQ status to check FIFO state
        regmap_read(i2s->regmap, IRQ_STATUS, &fifo_status);

        // Stop if RX FIFO is empty
        if (fifo_status & IRQ_RX_FIFO_EMPTY)
            break;

        // Read one frame from RX FIFO (assumes 32-bit FIFO reads)
        // Store sample based on PCM format
        __u32 sample;
        regmap_read(i2s->regmap, RX_FIFO, &sample);
        switch (i2s->format) {
        case SNDRV_PCM_FORMAT_S8:
            ((int8_t *)i2s->rx_buf)[i2s->rx_pos] = sample >> 24; // Extract 8-bit
            break;
        case SNDRV_PCM_FORMAT_S16_LE:
            ((int16_t *)i2s->rx_buf)[i2s->rx_pos] = sample >> 16; // Extract 16-bit
            break;
        case SNDRV_PCM_FORMAT_S24_LE:
            ((int32_t *)i2s->rx_buf)[i2s->rx_pos] = sample & 0xFFFFFF00; // Store 24-bit
            break;
        case SNDRV_PCM_FORMAT_S32_LE:
            ((uint32_t *)i2s->rx_buf)[i2s->rx_pos] = sample; // Store 32-bit
            break;
        default:
            dev_err(i2s->dev, "Unsupported RX format: %d\n", i2s->format);
            return;
        }
        i2s->rx_pos++; // Advance to next frame

        // Log progress for debugging
        dev_dbg(i2s->dev, "RX: pos=%zu, len=%zu, format=%d\n", i2s->rx_pos, i2s->rx_len, i2s->format);
    }

    // Check for buffer overrun (debug)
    if (i2s->rx_pos > i2s->rx_len) {
        dev_err(i2s->dev, "RX overrun: pos=%zu, len=%zu\n", i2s->rx_pos, i2s->rx_len);
    }

    // Check if all frames are processed
    if (i2s->rx_pos >= i2s->rx_len) {
        i2s->rx_start = false; // Mark RX as complete
        // Disable all RX interrupts
        regmap_update_bits(i2s->regmap, IRQ_ENABLE, IRQ_RX_READY | IRQ_RX_FIFO_FULL, 0);
        // Disable RX block
        regmap_update_bits(i2s->regmap, RX_CONFIG, RX_ENABLE_MASK, 0);
    }
}

/* Interrupt handler for TX and RX events */
static irqreturn_t rjn_i2s_irq_handler(int irq, void *dev_id)
{
    struct rjn_i2s_dev *i2s = dev_id; // Get driver data
    __u32 status; // IRQ status register value

    // Read IRQ status register
    regmap_read(i2s->regmap, IRQ_STATUS, &status);

    // Log IRQ status for debugging
    dev_dbg(i2s->dev, "IRQ status: 0x%x\n", status);

    // Clear all relevant interrupt bits to prevent storms
    __u32 clear_status = status & (IRQ_TX_FIFO_EMPTY | IRQ_TX_FIFO_TT | IRQ_RX_READY | IRQ_RX_FIFO_FULL);
    if (clear_status) {
        regmap_write(i2s->regmap, IRQ_STATUS, clear_status);
    }

    // Handle TX_FIFO_EMPTY interrupt (TX FIFO needs data)
    if ((status & IRQ_TX_FIFO_EMPTY) && i2s->tx_start) {
        rjn_i2s_fill_txfifo(i2s); // Fill TX FIFO
        return IRQ_HANDLED; // Interrupt processed
    }

    // Handle TX_FIFO_TT interrupt (threshold trigger for TX FIFO)
    if ((status & IRQ_TX_FIFO_TT) && i2s->tx_start) {
        rjn_i2s_fill_txfifo(i2s); // Fill TX FIFO
        return IRQ_HANDLED; // Interrupt processed
    }

    // Handle RX_READY interrupt (RX FIFO has data)
    if ((status & IRQ_RX_READY) && i2s->rx_start) {
        rjn_i2s_drain_rxfifo(i2s); // Drain RX FIFO
        return IRQ_HANDLED; // Interrupt processed
    }

    // Handle RX_FIFO_FULL interrupt (RX FIFO is full)
    if ((status & IRQ_RX_FIFO_FULL) && i2s->rx_start) {
        rjn_i2s_drain_rxfifo(i2s); // Drain RX FIFO
        return IRQ_HANDLED; // Interrupt processed
    }

    // No relevant interrupts handled
    return IRQ_NONE;
}

/* ALSA Trigger: Starts or stops playback/capture */
static int rjn_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
    struct rjn_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai); // Get driver data
    int ret = 0; // Return value

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        // Validate PCM buffer
        if (!substream->runtime->dma_area || substream->runtime->buffer_size == 0) {
            dev_err(i2s->dev, "Invalid PCM buffer\n");
            return -EINVAL;
        }

        // Enable master clock for I2S operation
        ret = clk_prepare_enable(i2s->mclk);
        if (ret) {
            dev_err(i2s->dev, "Failed to enable MCLK: %d\n", ret);
            return ret;
        }

        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            // Setup playback (TX)
            i2s->tx_buf = substream->runtime->dma_area; // Set TX buffer pointer
            // Calculate number of frames: buffer_size / (sample_bytes * channels)
            i2s->tx_len = substream->runtime->buffer_size /
                          ((snd_pcm_format_width(substream->runtime->format) / 8) * i2s->channels);
            i2s->tx_pos = 0; // Reset TX frame index
            i2s->tx_start = true; // Mark TX as active

            // Validate buffer alignment to frame size
            if (substream->runtime->buffer_size % ((snd_pcm_format_width(substream->runtime->format) / 8) * i2s->channels) != 0) {
                dev_err(i2s->dev, "Buffer size not aligned to frame size\n");
                clk_disable_unprepare(i2s->mclk);
                return -EINVAL;
            }

            // Pre-fill TX FIFO to reduce initial playback latency
            rjn_i2s_fill_txfifo(i2s); // No lock, assumes single-core serialization

            // Enable TX interrupts (empty and threshold)
            regmap_update_bits(i2s->regmap, IRQ_ENABLE,
                               IRQ_TX_FIFO_EMPTY | IRQ_TX_FIFO_TT,
                               IRQ_TX_FIFO_EMPTY | IRQ_TX_FIFO_TT);
            // Enable TX block
            regmap_update_bits(i2s->regmap, TX_CONFIG, TX_ENABLE_MASK, TX_ENABLE_MASK);

            // Log TX configuration for debugging
            dev_dbg(i2s->dev, "TX: buffer_size=%lu, format_width=%d, channels=%u, tx_len=%zu\n",
                    substream->runtime->buffer_size,
                    snd_pcm_format_width(substream->runtime->format),
                    i2s->channels, i2s->tx_len);
        } else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
            // Setup capture (RX)
            // Validate channel count
            if (i2s->channels == 0) {
                dev_err(i2s->dev, "Channel count not set\n");
                clk_disable_unprepare(i2s->mclk);
                return -EINVAL;
            }
            i2s->rx_buf = substream->runtime->dma_area; // Set RX buffer pointer
            // Calculate number of frames: buffer_size / (sample_bytes * channels)
            i2s->rx_len = substream->runtime->buffer_size /
                          ((snd_pcm_format_width(substream->runtime->format) / 8) * i2s->channels);
            i2s->rx_pos = 0; // Reset RX frame index
            i2s->rx_start = true; // Mark RX as active

            // Validate buffer alignment to frame size
            if (substream->runtime->buffer_size % ((snd_pcm_format_width(substream->runtime->format) / 8) * i2s->channels) != 0) {
                dev_err(i2s->dev, "Buffer size not aligned to frame size\n");
                clk_disable_unprepare(i2s->mclk);
                return -EINVAL;
            }

            // Enable RX interrupts (ready and full)
            regmap_update_bits(i2s->regmap, IRQ_ENABLE,
                               IRQ_RX_READY | IRQ_RX_FIFO_FULL,
                               IRQ_RX_READY | IRQ_RX_FIFO_FULL);
            // Enable RX block
            regmap_update_bits(i2s->regmap, RX_CONFIG, RX_ENABLE_MASK, RX_ENABLE_MASK);

            // Log RX configuration for debugging
            dev_dbg(i2s->dev, "RX: buffer_size=%lu, format_width=%d, channels=%u, rx_len=%zu\n",
                    substream->runtime->buffer_size,
                    snd_pcm_format_width(substream->runtime->format),
                    i2s->channels, i2s->rx_len);
        } else {
            clk_disable_unprepare(i2s->mclk); // Clean up clock on invalid stream
            return -EINVAL;
        }
        break;

    case SNDRV_PCM_TRIGGER_STOP:
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            // Stop playback (TX)
            i2s->tx_start = false; // Mark TX as inactive
            // Disable TX interrupts
            regmap_update_bits(i2s->regmap, IRQ_ENABLE, IRQ_TX_FIFO_EMPTY | IRQ_TX_FIFO_TT, 0);
            // Disable TX block
            regmap_update_bits(i2s->regmap, TX_CONFIG, TX_ENABLE_MASK, 0);
        } else {
            // Stop capture (RX)
            i2s->rx_start = false; // Mark RX as inactive
            // Disable RX interrupts
            regmap_update_bits(i2s->regmap, IRQ_ENABLE, IRQ_RX_READY | IRQ_RX_FIFO_FULL, 0);
            // Disable RX block
            regmap_update_bits(i2s->regmap, RX_CONFIG, RX_ENABLE_MASK, 0);
        }
        // Disable master clock
        clk_disable_unprepare(i2s->mclk);
        break;

    default:
        ret = -EINVAL; // Invalid command
        break;
    }

    return ret;
}

/* Configure I2S hardware parameters */
static int rjn_i2s_dai_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params,
                                 struct snd_soc_dai *dai)
{
    struct rjn_i2s_dev *i2s = to_info(dai); // Get driver data
    unsigned int val_tx = 0, val_rx = 0;    // TX/RX configuration values
    unsigned int mclk_rate, bclk_rate, div_bclk; // Clock configuration variables

    // Store channel count and format
    i2s->channels = params_channels(params);
    i2s->format = params_format(params);

    // Configure clocks if in master mode
    if (i2s->is_master_mode) {
        mclk_rate = clk_get_rate(i2s->mclk); // Get master clock rate
        bclk_rate = i2s->bclk_ratio * params_rate(params); // Calculate bit clock rate
        if (!bclk_rate)
            return -EINVAL; // Invalid bit clock rate
        div_bclk = DIV_ROUND_CLOSEST(mclk_rate, bclk_rate); // Compute clock divider
        // Update TX prescaler (MCLK and I2S divider)
        regmap_update_bits(i2s->regmap, TX_PRESCALER,
                           TX_MCLK_DIV_MASK | TX_I2S_DIV_MASK,
                           (0x01 << TX_MCLK_DIV) | (div_bclk << TX_I2S_DIV));
        // Update RX prescaler (MCLK and I2S divider)
        regmap_update_bits(i2s->regmap, RX_PRESCALER,
                           RX_MCLK_DIV_MASK | RX_I2S_DIV_MASK,
                           (0x01 << RX_MCLK_DIV) | (div_bclk << RX_I2S_DIV));

        // Log clock configuration for debugging
        dev_dbg(i2s->dev, "MCLK: %u Hz, BCLK: %u Hz, Divider: %u\n",
                mclk_rate, bclk_rate, div_bclk);
    }

    // Configure audio format based on PCM parameters
    switch (params_format(params)) {
    case SNDRV_PCM_FORMAT_S8:
        val_tx |= (_8_BIT << TX_DATA_WIDTH);
        val_rx |= (_8_BIT << RX_DATA_WIDTH);
        val_tx |= (_8_BIT << TX_CHANNEL_WIDTH);
        val_rx |= (_8_BIT << RX_CHANNEL_WIDTH);
        break;
    case SNDRV_PCM_FORMAT_S16_LE:
        val_tx |= (_16_BIT << TX_DATA_WIDTH);
        val_rx |= (_16_BIT << RX_DATA_WIDTH);
        val_tx |= (_16_BIT << TX_CHANNEL_WIDTH);
        val_rx |= (_16_BIT << RX_CHANNEL_WIDTH);
        break;
    case SNDRV_PCM_FORMAT_S24_LE:
        val_tx |= (_24_BIT << TX_DATA_WIDTH);
        val_rx |= (_24_BIT << RX_DATA_WIDTH);
        val_tx |= (_24_BIT << TX_CHANNEL_WIDTH);
        val_rx |= (_24_BIT << RX_CHANNEL_WIDTH);
        break;
    case SNDRV_PCM_FORMAT_S32_LE:
        val_tx |= (_32_BIT << TX_DATA_WIDTH);
        val_rx |= (_32_BIT << RX_DATA_WIDTH);
        val_tx |= (_32_BIT << TX_CHANNEL_WIDTH);
        val_rx |= (_32_BIT << RX_CHANNEL_WIDTH);
        break;
    default:
        return -EINVAL; // Unsupported format
    }

    // Configure number of channels
    switch (params_channels(params)) {
    case 8:
        val_tx |= (_8_CHANNEL << TX_NUM_CHANNELS);
        val_rx |= (_8_CHANNEL << RX_NUM_CHANNELS);
        break;
    case 6:
        val_tx |= (_6_CHANNEL << TX_NUM_CHANNELS);
        val_rx |= (_6_CHANNEL << RX_NUM_CHANNELS);
        break;
    case 4:
        val_tx |= (_4_CHANNEL << TX_NUM_CHANNELS);
        val_rx |= (_4_CHANNEL << RX_NUM_CHANNELS);
        break;
    case 2:
        val_tx |= (_2_CHANNEL << TX_NUM_CHANNELS);
        val_rx |= (_2_CHANNEL << RX_NUM_CHANNELS);
        break;
    case 1:
        val_tx |= (_1_CHANNEL << TX_NUM_CHANNELS);
        val_rx |= (_1_CHANNEL << RX_NUM_CHANNELS);
        break;
    default:
        dev_err(i2s->dev, "Invalid channel count: %d\n", params_channels(params));
        return -EINVAL;
    }

    // Common fields
    val_tx |= (I2S_MODE << TX_PROTO_MODE) | (MSB_BIT_ORDER << TX_BIT_ORDER) | (1 << TX_PADDING);
    val_rx |= (I2S_MODE << RX_PROTO_MODE) | (MSB_BIT_ORDER << RX_BIT_ORDER);

    // Final register write
    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
        regmap_update_bits(i2s->regmap, RX_CONFIG,
                           RX_NUM_CHANNELS_MASK | RX_DATA_WIDTH_MASK |
                           RX_CHANNEL_WIDTH_MASK | RX_PROTO_MODE_MASK |
                           RX_BIT_ORDER_MASK,
                           val_rx);
    } else {
        regmap_update_bits(i2s->regmap, TX_CONFIG,
                           TX_NUM_CHANNELS_MASK | TX_DATA_WIDTH_MASK |
                           TX_CHANNEL_WIDTH_MASK | TX_PROTO_MODE_MASK |
                           TX_BIT_ORDER_MASK | TX_PADDING_MASK,
                           val_tx);
    }

    return 0;
}

/* Set bit clock ratio (BCLK cycles per frame) */
int rjn_i2s_dai_bclk_ratio(struct snd_soc_dai *dai, unsigned int ratio)
{
    struct rjn_i2s_dev *i2s = to_info(dai);
    i2s->bclk_ratio = ratio;
    return 0;
}

/* Set system clock (MCLK) frequency */
int rjn_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
    int ret;
    struct rjn_i2s_dev *i2s = to_info(dai);

    if (freq == 0)
        return 0;

    // Set clock rate to the specified frequency
    ret = clk_set_rate(i2s->mclk, freq);
    if (ret)
        dev_err(i2s->dev, "Fail to set mclk %d\n", ret);

    return ret;
}

/* Set audio format (clock mode, polarity, justification) */
int rjn_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    struct rjn_i2s_dev *i2s = to_info(dai);
    int ret = 0;
    unsigned int val_tx = 0, val_rx = 0;

    /* Clock Mode: Master or Slave */
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

    regmap_update_bits(i2s->regmap, TX_CONFIG, TX_CLK_MODE_MASK, val_tx);
    regmap_update_bits(i2s->regmap, RX_CONFIG, RX_CLK_MODE_MASK, val_rx);

    /* Clock Polarity */
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
    case SND_SOC_DAIFMT_NB_NF:  // Normal BCLK
        val_tx = (0 << TX_CLK_POL);
        val_rx = (0 << RX_CLK_POL);
        break;
    case SND_SOC_DAIFMT_IB_IF:  // Inverted BCLK + Inverted LRCLK
        val_tx = (1 << TX_CLK_POL);
        val_rx = (1 << RX_CLK_POL);
        break;
    default:
        dev_warn(i2s->dev, "Frame clock (LRCLK) inversion not supported, ignoring\n");
        return -EINVAL;
    }

    regmap_update_bits(i2s->regmap, TX_CONFIG, TX_CLK_POL_MASK, val_tx);
    regmap_update_bits(i2s->regmap, RX_CONFIG, RX_CLK_POL_MASK, val_rx);

    /* Justification & WS Delay */
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

    regmap_update_bits(i2s->regmap, TX_CONFIG, TX_JUSTIFICATION_MASK | TX_SD_DELAY_MASK, val_tx);
    regmap_update_bits(i2s->regmap, RX_CONFIG, RX_JUSTIFICATION_MASK | RX_SD_DELAY_MASK, val_rx);

    return ret;
}

/* DAI operations */
static const struct snd_soc_dai_ops rjn_i2s_dai_ops = {
    .hw_params = rjn_i2s_dai_hw_params,
    .set_bclk_ratio = rjn_i2s_dai_bclk_ratio,
    .set_sysclk = rjn_i2s_set_sysclk,
    .set_fmt = rjn_i2s_set_fmt,
    .trigger = rjn_i2s_trigger,
};

/* DAI Driver Definition */
static struct snd_soc_dai_driver rjn_i2s_dai = {
    .name = "rjn-i2s-dai",
    .playback = {
        .stream_name = "Playback",
        .channels_min = 1,
        .channels_max = 8,
        .rates = SNDRV_PCM_RATE_8000_192000,
        .formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
    },
    .capture = {
        .stream_name = "Capture",
        .channels_min = 1,
        .channels_max = 8,
        .rates = SNDRV_PCM_RATE_8000_192000,
        .formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
    },
    .ops = &rjn_i2s_dai_ops,
    .symmetric_rate = 1,
};

/* ALSA SoC Component Driver */
static const struct snd_soc_component_driver rjn_i2s_cmpt_drv = {
    .name = DRV_NAME,
    .legacy_dai_naming = 0, // Modern naming convention
};

/* Register Defaults */
static const struct reg_default rjn_i2s_reg_defaults[] = {
    { 0x00, 0xF3C4 },
    { 0x04, 0xF3C4 },
    { 0x08, 0x2F0 },
    { 0x0c, 0x2F0 },
};

/* Check if register is writeable */
static bool rjn_i2s_writeable_regs(struct device *dev, unsigned int reg)
{
    switch (reg) {
    case TX_CONFIG:
    case RX_CONFIG:
    case TX_PRESCALER:
    case RX_PRESCALER:
    case IRQ_ENABLE:
    case IRQ_STATUS:
    case FIFO_FLUSH:
    case TX_FIFO:
        return true;
    default:
        return false;
    }
}

/* Check if register is readable */
static bool rjn_i2s_readable_regs(struct device *dev, unsigned int reg)
{
    switch (reg) {
    case TX_CONFIG:
    case RX_CONFIG:
    case TX_PRESCALER:
    case RX_PRESCALER:
    case IRQ_ENABLE:
    case IRQ_STATUS:
    case FIFO_STATUS:
    case RX_FIFO:
        return true;
    default:
        return false;
    }
}

/* Regmap Configuration */
static const struct regmap_config rjn_i2s_regmap_config = {
    .reg_bits = 32,
    .reg_stride = 4,
    .val_bits = 32,
    .max_register = SOC_TIMEOUT,
    .writeable_reg = rjn_i2s_writeable_regs,
    .readable_reg = rjn_i2s_readable_regs,
};

/* Device Tree Match Table */
static const struct of_device_id rjn_i2s_match[] = {
    { .compatible = "ramanujan,rjn-i2s" },
    { .compatible = "ramanujan,rjn-i2s0" },
    {},
};

/* Initialize I2S controller */
static void rjn_i2s_init(struct rjn_i2s_dev *i2s_dev)
{
    // Enable TX and RX blocks
    regmap_update_bits(i2s_dev->regmap, TX_CONFIG, TX_ENABLE_MASK, TX_ENABLE_MASK);
    regmap_update_bits(i2s_dev->regmap, RX_CONFIG, RX_ENABLE_MASK, RX_ENABLE_MASK);

    // Set FIFO threshold values
    regmap_update_bits(i2s_dev->regmap, FIFO_THRESHOLD,
                       RX_FIFO_THRESHOLD_MASK | TX_FIFO_THRESHOLD_MASK,
                       0x40 << RX_FIFO_THRESHOLD | 0x40 << TX_FIFO_THRESHOLD);

    // Set SOC_TIMEOUT value
    regmap_write(i2s_dev->regmap, SOC_TIMEOUT, 0x40);
}

/* Probe Function: Initializes the I2S Controller */
static int rjn_i2s_probe(struct platform_device *pdev)
{
    struct rjn_i2s_dev *i2s_dev;
    struct resource *res;
    void __iomem *regs;
    int ret;

    // Allocate memory for private data structure
    i2s_dev = devm_kzalloc(&pdev->dev, sizeof(*i2s_dev), GFP_KERNEL);
    if (!i2s_dev)
        return -ENOMEM;

    i2s_dev->dev = &pdev->dev;

    // Get base address from platform device resource
    regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
    if (IS_ERR(regs)) {
        ret = PTR_ERR(regs);
        goto err_clk;
    }

    // Clock configuration
    i2s_dev->mclk = devm_clk_get(&pdev->dev, "i2s_clock");
    if (IS_ERR(i2s_dev->mclk)) {
        dev_err(&pdev->dev, "Can't retrieve I2S master clock\n");
        return PTR_ERR(i2s_dev->mclk);
    }

    // Prepare and enable master clock
    ret = clk_prepare_enable(i2s_dev->mclk);
    if (ret) {
        dev_err(i2s_dev->dev, "mclk enable failed %d\n", ret);
        return ret;
    }

    // Request IRQ
    i2s_dev->irq = platform_get_irq(pdev, 0);
    if (i2s_dev->irq < 0) {
        dev_err(&pdev->dev, "request irq error\n");
        return ret;
    }

    ret = devm_request_irq(&pdev->dev, i2s_dev->irq, rjn_i2s_irq_handler, 0, dev_name(&pdev->dev), i2s_dev);
    if (ret < 0) {
        dev_err(i2s_dev->dev, "failed to register irq\n");
        return ret;
    }

    // Bind base address to private structure
    i2s_dev->regs = regs;

    // Initialize managed register map
    i2s_dev->regmap = devm_regmap_init_mmio(&pdev->dev, regs, &rjn_i2s_regmap_config);
    if (IS_ERR(i2s_dev->regmap)) {
        dev_err(&pdev->dev, "Failed to initialise managed register map\n");
        ret = PTR_ERR(i2s_dev->regmap);
        goto err_clk;
    }

    i2s_dev->dev = &pdev->dev;
    i2s_dev->is_master_mode = true; // Set default to master mode
    spin_lock_init(&i2s_dev->lock); // Initialize spinlock for future use

    // Set platform driver data
    platform_set_drvdata(pdev, i2s_dev);

    // Initialize I2S controller
    rjn_i2s_init(i2s_dev);

    // Register component and DAI with ALSA
    ret = devm_snd_soc_register_component(&pdev->dev, &rjn_i2s_cmpt_drv, &rjn_i2s_dai, 1);
    if (ret) {
        dev_err(&pdev->dev, "Could not register DAI\n");
        goto err_suspend;
    }

    return 0;

err_suspend:
err_clk:
    clk_disable_unprepare(i2s_dev->mclk);
    return ret;
}

/* Remove Function: Cleans up resources */
static int rjn_i2s_remove(struct platform_device *pdev)
{
    struct rjn_i2s_dev *i2s_dev = platform_get_drvdata(pdev);
    clk_disable_unprepare(i2s_dev->mclk);
    return 0;
}

/* Module Information */
MODULE_DEVICE_TABLE(of, rjn_i2s_match);

static struct platform_driver rjn_i2s_driver = {
    .probe = rjn_i2s_probe,
    .remove = rjn_i2s_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = of_match_ptr(rjn_i2s_match),
    },
};

module_platform_driver(rjn_i2s_driver);

MODULE_AUTHOR("Ramanujan : RJN I2S DRIVER");
MODULE_DESCRIPTION("An I2S Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);