#ifndef SDIPS_I2S_DEFINES_H
#define SDIPS_I2S_DEFINES_H
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
// Date Created   : 02-10-2025

// Generator      : IIP Compiler Version 1.1
//0---------------------------------------------------------------------------------------------------

//0###################################################################################################
// Defines for i2s 
//0###################################################################################################
#define SDIPS_I2S_TX_CONFIG                                0x0       // This is transmit configuration register
#define SDIPS_I2S_RX_CONFIG                                0x4       // This is receive configuration register
#define SDIPS_I2S_TX_PRESCALER                             0x8       // This is transmit clock divider
#define SDIPS_I2S_RX_PRESCALER                             0xC       // This is receive clock divider
#define SDIPS_I2S_IRQ_ENABLE                               0x10      // This register is IRQ enable register, used to enable interrupts
#define SDIPS_I2S_IRQ_STATUS                               0x14      // This IRQ status register, write 1 to clear interrupt
#define SDIPS_I2S_FIFO_FLUSH                               0x18      // Register to flush FIFO's
#define SDIPS_I2S_FIFO_THRESHOLD                           0x1C      // Register to control threshold for interrupt
#define SDIPS_I2S_FIFO_STATUS                              0x20      // Register for transmit and receive FIFO status
#define SDIPS_I2S_TX_FIFO                                  0x24      // Transmit data FIFO(write)
#define SDIPS_I2S_RX_FIFO                                  0x28      // Receive data FIFO(read)
#define SDIPS_I2S_SOC_TIMEOUT                              0x2C      // SOC timeout register

//0###################################################################################################
// Defines for subfields of CSR TX_CONFIG 
//0###################################################################################################
#define SDIPS_I2S_TX_ENABLE                                0         // Transmitter enable
#define SDIPS_I2S_TX_NUM_CHANNELS                          5:1       // Controls number of transmit channels, 0 means 1 channel
#define SDIPS_I2S_TX_CHANNEL_WIDTH                         11:6      // Controls transmit channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define SDIPS_I2S_TX_DATA_WIDTH                            17:12     // Controls transmit data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define SDIPS_I2S_TX_PADDING                               18        // Controls bit value to use in padding
#define SDIPS_I2S_TX_PROTO_MODE                            20:19     // Controls protocol mode
#define SDIPS_I2S_TX_BIT_ORDER                             21        // Controls bit ordering
#define SDIPS_I2S_TX_JUSTIFICATION                         22        // Controls justification, valid only in I2S mode
#define SDIPS_I2S_TX_SD_DELAY                              25:23     // Controls WS to WD delay in SCK
#define SDIPS_I2S_TX_CLK_MODE                              26        // Controls transmit clock mode
#define SDIPS_I2S_TX_CLK_POL                               27        // Transmit clock polarity

//0###################################################################################################
// Defines for subfields of CSR RX_CONFIG 
//0###################################################################################################
#define SDIPS_I2S_RX_ENABLE                                0         // Receiver enable
#define SDIPS_I2S_RX_NUM_CHANNELS                          5:1       // Controls number of receive channels, 0 means 1 channel
#define SDIPS_I2S_RX_CHANNEL_WIDTH                         11:6      // Controls receive channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define SDIPS_I2S_RX_DATA_WIDTH                            17:12     // Controls receive data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define SDIPS_I2S_RX_PROTO_MODE                            19:18     // Controls protocol mode
#define SDIPS_I2S_RX_BIT_ORDER                             20        // Controls bit ordering
#define SDIPS_I2S_RX_JUSTIFICATION                         21        // Controls justification, valid only in I2S mode
#define SDIPS_I2S_RX_SD_DELAY                              24:22     // Controls WS to WD delay in SCK
#define SDIPS_I2S_RX_CLK_MODE                              25        // Controls receive clock mode
#define SDIPS_I2S_RX_CLK_POL                               26        // Receive clock polarity

//0###################################################################################################
// Defines for subfields of CSR TX_PRESCALER 
//0###################################################################################################
#define SDIPS_I2S_TX_MCLK_DIV                              15:0      // This is Master clock divider, it divides i_clk to generate o_i2s_tx_mck
#define SDIPS_I2S_TX_I2S_DIV                               23:16     // This is I2S clock divider, it divides o_i2s_tx_mck to generate o_i2s_tx_sck

//0###################################################################################################
// Defines for subfields of CSR RX_PRESCALER 
//0###################################################################################################
#define SDIPS_I2S_RX_MCLK_DIV                              15:0      // This is Master clock divider, it divides i_clk to generate o_i2s_rx_mck
#define SDIPS_I2S_RX_I2S_DIV                               23:16     // This is I2S clock divider, it divides o_i2s_rx_mck to generate o_i2s_rx_sck

//0###################################################################################################
// Defines for subfields of CSR IRQ_ENABLE 
//0###################################################################################################
#define SDIPS_I2S_IRQ_RX_READY                             0         // Receive FIFO has data
#define SDIPS_I2S_IRQ_TX_FIFO_TT                           1         // Transmit FIFO threshold reached on read
#define SDIPS_I2S_IRQ_TX_FIFO_UR                           2         // Transmit FIFO underrun
#define SDIPS_I2S_IRQ_TX_FIFO_OR                           3         // Transmit FIFO overrun
#define SDIPS_I2S_IRQ_TX_FIFO_FULL                         4         // Transmit FIFO full
#define SDIPS_I2S_IRQ_TX_FIFO_EMPTY                        5         // Transmit FIFO empty
#define SDIPS_I2S_IRQ_RX_FIFO_TT                           6         // Receive FIFO threshold reached on write
#define SDIPS_I2S_IRQ_RX_FIFO_UR                           7         // Receive FIFO underrun
#define SDIPS_I2S_IRQ_RX_FIFO_OR                           8         // Receive FIFO overrun                 
#define SDIPS_I2S_IRQ_RX_FIFO_FULL                         9         // Receive FIFO full
#define SDIPS_I2S_IRQ_RX_FIFO_EMPTY                        10        // Receive FIFO empty

//0###################################################################################################
// Defines for subfields of CSR FIFO_FLUSH 
//0###################################################################################################
#define SDIPS_I2S_TX_FIFO_FLUSH                            0         // Transmit Data FIFO flush
#define SDIPS_I2S_RX_FIFO_FLUSH                            1         // Receive Data FIFO flush

//0###################################################################################################
// Defines for subfields of CSR FIFO_THRESHOLD 
//0###################################################################################################
#define SDIPS_I2S_TX_FIFO_THRESHOLD                        7:0       // Transmit FIFO threshold for read
#define SDIPS_I2S_RX_FIFO_THRESHOLD                        15:8      // Receive FIFO threshold for write

//0###################################################################################################
// Defines for subfields of CSR FIFO_STATUS 
//0###################################################################################################
#define SDIPS_I2S_TX_FIFO_DEPTH                            6:0       // Transmit FIFO depth
#define SDIPS_I2S_RX_FIFO_DEPTH                            13:7      // Receive FIFO depth
#define SDIPS_I2S_TX_FIFO_AVAIL                            20:14     // Transmit FIFO free space
#define SDIPS_I2S_RX_FIFO_USED                             27:21     // Receive FIFO used space
#define SDIPS_I2S_RX_FIFO_FULL                             28        // Receive FIFO full
#define SDIPS_I2S_RX_FIFO_EMPTY                            29        // Receive FIFO empty
#define SDIPS_I2S_TX_FIFO_FULL                             30        // Transmit FIFO full
#define SDIPS_I2S_TX_FIFO_EMPTY                            31        // Transmit FIFO empty

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_NUM_CHANNELS of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_CHANNEL_WIDTH of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_DATA_WIDTH of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_PADDING of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_PROTO_MODE of i2s 
//0###################################################################################################
#define SDIPS_I2S_I2S                                      0         // I2S mode of operation
#define SDIPS_I2S_TDM                                      1         // TDM/DSP/TDM mode of operation

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_BIT_ORDER of i2s 
//0###################################################################################################
#define SDIPS_I2S_LSB                                      0         // Least sigification bit first
#define SDIPS_I2S_MSB                                      1         // Most sigification bit first

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_JUSTIFICATION of i2s 
//0###################################################################################################
#define SDIPS_I2S_LEFT_JUST                                0         // I2S left justified mode
#define SDIPS_I2S_RIGHT_JUST                               1         // I2S right justified mode

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_SD_DELAY of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_CLK_MODE of i2s 
//0###################################################################################################
#define SDIPS_I2S_MASTER                                   0         // Clock is driven by core, Master transmitter mode
#define SDIPS_I2S_SLAVE                                    1         // Clock is sampled by core, Slave transmitter mode

//0###################################################################################################
// Defines for enum for csr TX_CONFIG field TX_CLK_POL of i2s 
//0###################################################################################################
#define SDIPS_I2S_NEDGE                                    0         // Drive on negative edge
#define SDIPS_I2S_PEDGE                                    1         // Drive on positive edge

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_NUM_CHANNELS of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_CHANNEL_WIDTH of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_DATA_WIDTH of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_PROTO_MODE of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_BIT_ORDER of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_JUSTIFICATION of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_SD_DELAY of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_CLK_MODE of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_CONFIG field RX_CLK_POL of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr TX_PRESCALER field TX_MCLK_DIV of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr TX_PRESCALER field TX_I2S_DIV of i2s 
//0###################################################################################################

//0###################################################################################################
// Defines for enum for csr RX_PRESCALER field RX_MCLK_DIV of i2s 
//0###################################################################################################


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
#endif
