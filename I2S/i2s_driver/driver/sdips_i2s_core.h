#ifndef SDIPS_I2S_CORE_H
#define SDIPS_I2S_CORE_H
//0---------------------------------------------------------------------------------------------------
//                           SmartDV IP Solutions Proprietary 
//            Copyright 2007-2025 SmartDV IP Solutions India Private Limited
// 
//                               CONFIDENTIAL INFORMATION
// 
//                                  All rights reserved
// 
//             The use, modification, or duplication of this product is protected 
//             according to SmartDV IP Solution's licensing agreement.
// 
//             This Intellectual property contains confidential and proprietary 
//             information which are the properties of SmartDV IP Solution's. 
// 
//             Unauthorized use, disclosure, duplication, or reproduction are prohibited. 
//0---------------------------------------------------------------------------------------------------
// Defines for all CSR in IIP
//0-------------------------------------------------------------------------------
#define TX_CONFIG                       0x0       // This is transmit configuration register
#define RX_CONFIG                       0x4       // This is receive configuration register
#define TX_PRESCALER                    0x8       // This is transmit clock divider
#define RX_PRESCALER                    0xC       // This is receive clock divider
#define IRQ_ENABLE                      0x10      // This register is IRQ enable register, used to enable interrupts
#define IRQ_STATUS                                // 
#define FIFO_FLUSH                      0x18      // Register to flush FIFO's
#define FIFO_THRESHOLD                  0x1C      // Register to control threshold for interrupt
#define FIFO_STATUS                     0x20      // Register for transmit and receive FIFO status
#define TX_FIFO                         0x24      // Transmit data FIFO(write)
#define RX_FIFO                         0x28      // Receive data FIFO(read)
#define SOC_TIMEOUT                     0x2C      // SOC timeout register
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR TX_CONFIG
//0-------------------------------------------------------------------------------
#define TX_ENABLE                       0         // Transmitter enable
#define TX_NUM_CHANNELS                 1         // Controls number of transmit channels, 0 means 1 channel
#define TX_CHANNEL_WIDTH 6         // Controls transmit channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define TX_DATA_WIDTH                   12        // Controls transmit data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define TX_PADDING                      18        // Controls bit value to use in padding
#define TX_PROTO_MODE                   19        // Controls protocol mode
#define TX_BIT_ORDER                    21        // Controls bit ordering
#define TX_JUSTIFICATION 22        // Controls justification, valid only in I2S mode
#define TX_SD_DELAY                     23        // Controls WS to WD delay in SCK
#define TX_CLK_MODE                     26        // Controls transmit clock mode
#define TX_CLK_POL                      27        // Transmit clock polarity
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR RX_CONFIG
//0-------------------------------------------------------------------------------
#define RX_ENABLE                       0         // Receiver enable
#define RX_NUM_CHANNELS                 1         // Controls number of receive channels, 0 means 1 channel
#define RX_CHANNEL_WIDTH 6         // Controls receive channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define RX_DATA_WIDTH                   12        // Controls receive data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define RX_PROTO_MODE                   18        // Controls protocol mode
#define RX_BIT_ORDER                    20        // Controls bit ordering
#define RX_JUSTIFICATION 21        // Controls justification, valid only in I2S mode
#define RX_SD_DELAY                     22        // Controls WS to WD delay in SCK
#define RX_CLK_MODE                     25        // Controls receive clock mode
#define RX_CLK_POL                      26        // Receive clock polarity
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR TX_PRESCALER
//0-------------------------------------------------------------------------------
#define TX_MCLK_DIV                     0         // This is Master clock divider, it divides i_clk to generate o_i2s_tx_mck
#define TX_I2S_DIV                      16        // This is I2S clock divider, it divides o_i2s_tx_mck to generate o_i2s_tx_sck
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR RX_PRESCALER
//0-------------------------------------------------------------------------------
#define RX_MCLK_DIV                     0         // This is Master clock divider, it divides i_clk to generate o_i2s_rx_mck
#define RX_I2S_DIV                      16        // This is I2S clock divider, it divides o_i2s_rx_mck to generate o_i2s_rx_sck
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR IRQ_ENABLE
//0-------------------------------------------------------------------------------
#define IRQ_RX_READY                    0         // Receive FIFO has data
#define IRQ_TX_FIFO_TT                  1         // Transmit FIFO threshold reached on read
#define IRQ_TX_FIFO_UR                  2         // Transmit FIFO underrun
#define IRQ_TX_FIFO_OR                  3         // Transmit FIFO overrun
#define IRQ_TX_FIFO_FULL 4         // Transmit FIFO full
#define IRQ_TX_FIFO_EMPTY 5         // Transmit FIFO empty
#define IRQ_RX_FIFO_TT                  6         // Receive FIFO threshold reached on write
#define IRQ_RX_FIFO_UR                  7         // Receive FIFO underrun
#define IRQ_RX_FIFO_OR                  8         // Receive FIFO overrun                 
#define IRQ_RX_FIFO_FULL 9         // Receive FIFO full
#define IRQ_RX_FIFO_EMPTY 10        // Receive FIFO empty
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_FLUSH
//0-------------------------------------------------------------------------------
#define TX_FIFO_FLUSH                   0         // Transmit Data FIFO flush
#define RX_FIFO_FLUSH                   1         // Receive Data FIFO flush
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_THRESHOLD
//0-------------------------------------------------------------------------------
#define TX_FIFO_THRESHOLD 0         // Transmit FIFO threshold for read
#define RX_FIFO_THRESHOLD 8         // Receive FIFO threshold for write
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_STATUS
//0-------------------------------------------------------------------------------
#define TX_FIFO_DEPTH                   0         // Transmit FIFO depth
#define RX_FIFO_DEPTH                   7         // Receive FIFO depth
#define TX_FIFO_AVAIL                   14        // Transmit FIFO free space
#define RX_FIFO_USED                    21        // Receive FIFO used space
#define RX_FIFO_FULL                    28        // Receive FIFO full
#define RX_FIFO_EMPTY                   29        // Receive FIFO empty
#define TX_FIFO_FULL                    30        // Transmit FIFO full
#define TX_FIFO_EMPTY                   31        // Transmit FIFO empty
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR TX_CONFIG - Bit widths
//0-------------------------------------------------------------------------------
#define TX_ENABLE_WIDTH                 1         // Transmitter enable
#define TX_NUM_CHANNELS_WIDTH 5         // Controls number of transmit channels, 0 means 1 channel
#define TX_CHANNEL_WIDTH_WIDTH 6         // Controls transmit channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define TX_DATA_WIDTH_WIDTH 6         // Controls transmit data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define TX_PADDING_WIDTH 1         // Controls bit value to use in padding
#define TX_PROTO_MODE_WIDTH 2         // Controls protocol mode
#define TX_BIT_ORDER_WIDTH 1         // Controls bit ordering
#define TX_JUSTIFICATION_WIDTH 1         // Controls justification, valid only in I2S mode
#define TX_SD_DELAY_WIDTH 3         // Controls WS to WD delay in SCK
#define TX_CLK_MODE_WIDTH 1         // Controls transmit clock mode
#define TX_CLK_POL_WIDTH 1         // Transmit clock polarity
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR RX_CONFIG - Bit widths
//0-------------------------------------------------------------------------------
#define RX_ENABLE_WIDTH                 1         // Receiver enable
#define RX_NUM_CHANNELS_WIDTH 5         // Controls number of receive channels, 0 means 1 channel
#define RX_CHANNEL_WIDTH_WIDTH 6         // Controls receive channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define RX_DATA_WIDTH_WIDTH 6         // Controls receive data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define RX_PROTO_MODE_WIDTH 2         // Controls protocol mode
#define RX_BIT_ORDER_WIDTH 1         // Controls bit ordering
#define RX_JUSTIFICATION_WIDTH 1         // Controls justification, valid only in I2S mode
#define RX_SD_DELAY_WIDTH 3         // Controls WS to WD delay in SCK
#define RX_CLK_MODE_WIDTH 1         // Controls receive clock mode
#define RX_CLK_POL_WIDTH 1         // Receive clock polarity
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR TX_PRESCALER - Bit widths
//0-------------------------------------------------------------------------------
#define TX_MCLK_DIV_WIDTH 16        // This is Master clock divider, it divides i_clk to generate o_i2s_tx_mck
#define TX_I2S_DIV_WIDTH 8         // This is I2S clock divider, it divides o_i2s_tx_mck to generate o_i2s_tx_sck
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR RX_PRESCALER - Bit widths
//0-------------------------------------------------------------------------------
#define RX_MCLK_DIV_WIDTH 16        // This is Master clock divider, it divides i_clk to generate o_i2s_rx_mck
#define RX_I2S_DIV_WIDTH 8         // This is I2S clock divider, it divides o_i2s_rx_mck to generate o_i2s_rx_sck
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR IRQ_ENABLE - Bit widths
//0-------------------------------------------------------------------------------
#define IRQ_RX_READY_WIDTH 1         // Receive FIFO has data
#define IRQ_TX_FIFO_TT_WIDTH 1         // Transmit FIFO threshold reached on read
#define IRQ_TX_FIFO_UR_WIDTH 1         // Transmit FIFO underrun
#define IRQ_TX_FIFO_OR_WIDTH 1         // Transmit FIFO overrun
#define IRQ_TX_FIFO_FULL_WIDTH 1         // Transmit FIFO full
#define IRQ_TX_FIFO_EMPTY_WIDTH 1         // Transmit FIFO empty
#define IRQ_RX_FIFO_TT_WIDTH 1         // Receive FIFO threshold reached on write
#define IRQ_RX_FIFO_UR_WIDTH 1         // Receive FIFO underrun
#define IRQ_RX_FIFO_OR_WIDTH 1         // Receive FIFO overrun                 
#define IRQ_RX_FIFO_FULL_WIDTH 1         // Receive FIFO full
#define IRQ_RX_FIFO_EMPTY_WIDTH 1         // Receive FIFO empty
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_FLUSH - Bit widths
//0-------------------------------------------------------------------------------
#define TX_FIFO_FLUSH_WIDTH 1         // Transmit Data FIFO flush
#define RX_FIFO_FLUSH_WIDTH 1         // Receive Data FIFO flush
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_THRESHOLD - Bit widths
//0-------------------------------------------------------------------------------
#define TX_FIFO_THRESHOLD_WIDTH 8         // Transmit FIFO threshold for read
#define RX_FIFO_THRESHOLD_WIDTH 8         // Receive FIFO threshold for write
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_STATUS - Bit widths
//0-------------------------------------------------------------------------------
#define TX_FIFO_DEPTH_WIDTH 7         // Transmit FIFO depth
#define RX_FIFO_DEPTH_WIDTH 7         // Receive FIFO depth
#define TX_FIFO_AVAIL_WIDTH 7         // Transmit FIFO free space
#define RX_FIFO_USED_WIDTH 7         // Receive FIFO used space
#define RX_FIFO_FULL_WIDTH 1         // Receive FIFO full
#define RX_FIFO_EMPTY_WIDTH 1         // Receive FIFO empty
#define TX_FIFO_FULL_WIDTH 1         // Transmit FIFO full
#define TX_FIFO_EMPTY_WIDTH 1         // Transmit FIFO empty



#endif
