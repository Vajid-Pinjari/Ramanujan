#ifndef SDIPS_I2S_BASIC_C
#define SDIPS_I2S_BASIC_C
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
// Description    : This is simple I2S transmitter/receiver driver 
// Generator      : IIP Compiler Version 1.1
//0---------------------------------------------------------------------------------------------------


#include <stdio.h>
#include <stdint.h>
#include "sdips_i2s_defines.h"
#include "sdips_i2s_basic.h"



//0---------------------------------------------------------------------------------------------------
// mask_irq :This method is used to mask the IRQ 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//            p_mask_21b :Interrupts to mask 
//0---------------------------------------------------------------------------------------------------
void mask_irq (
  struct sdips_i2s_config*                 p_config_st                                                ,
  uint32_t                  p_mask_21b                                                                
) {
  uint32_t                m_new_mask_21b                                                              ; // New register mask
  m_new_mask_21b             = read_reg(SDIPS_I2S_IRQ_ENABLE) & ~p_mask_21b;
  write_reg(SDIPS_I2S_IRQ_ENABLE,m_new_mask_21b);
}


//0---------------------------------------------------------------------------------------------------
// unmask_irq :This method is used to unmask the IRQ 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//            p_mask_21b :Interrupts to mask 
//0---------------------------------------------------------------------------------------------------
void unmask_irq (
  struct sdips_i2s_config*                 p_config_st                                                ,
  uint32_t                  p_mask_21b                                                                
) {
  uint32_t                m_new_mask_21b                                                              ; // New register mask
  m_new_mask_21b             = read_reg(SDIPS_I2S_IRQ_ENABLE) | p_mask_21b;
  write_reg(SDIPS_I2S_IRQ_ENABLE,m_new_mask_21b);
}


//0---------------------------------------------------------------------------------------------------
// init_chip :This method is used to initialize the I2S core 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void init_chip (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  //1-------------------------------------------------------------------------------------------------
  // Set transmit prescaler 
  //1-------------------------------------------------------------------------------------------------
  write_tx_prescaler_csr(p_config_st);
  //1-------------------------------------------------------------------------------------------------
  // Set transmit config 
  //1-------------------------------------------------------------------------------------------------
  write_tx_config_csr(p_config_st);
  //1-------------------------------------------------------------------------------------------------
  // Set receive prescaler 
  //1-------------------------------------------------------------------------------------------------
  write_rx_prescaler_csr(p_config_st);
  //1-------------------------------------------------------------------------------------------------
  // Set receive config 
  //1-------------------------------------------------------------------------------------------------
  write_rx_config_csr(p_config_st);
}


//0---------------------------------------------------------------------------------------------------
// enable_chip :This method is used to enable/disable chip 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//            p_enable_b :1 means enable, 0 means disable 
//0---------------------------------------------------------------------------------------------------
void enable_chip (
  struct sdips_i2s_config*                 p_config_st                                                ,
  uint8_t                   p_enable_b                                                                
) {
  p_config_st->tx_enable_b = p_enable_b;
  p_config_st->rx_enable_b = p_enable_b;
  //1-------------------------------------------------------------------------------------------------
  // Set transmit config 
  //1-------------------------------------------------------------------------------------------------
  write_tx_config_csr(p_config_st);
  //1-------------------------------------------------------------------------------------------------
  // Set transmit config 
  //1-------------------------------------------------------------------------------------------------
  write_rx_config_csr(p_config_st);
}


//0---------------------------------------------------------------------------------------------------
// reset_chip :This method is used to reset the chip 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void reset_chip (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  enable_chip(p_config_st,0);
  mask_irq(p_config_st,0xFF);
  enable_chip(p_config_st,1);
}


//0---------------------------------------------------------------------------------------------------
// enable_data_irq :This method is used to enable Data IRQ's 
//0---------------------------------------------------------------------------------------------------
//             parameter :No parameter 
//0---------------------------------------------------------------------------------------------------
void enable_data_irq (
) {
  uint32_t                m_data_21b                                                                  ; // Register value to write
  m_data_21b                 = 0xFFFF;
  write_reg(SDIPS_I2S_IRQ_ENABLE,m_data_21b); 
}


//0---------------------------------------------------------------------------------------------------
// send_sample :This method is used to transmit audio sample from transmitter 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//              p_cmd_st :Pointer to command object 
//0---------------------------------------------------------------------------------------------------
void send_sample (
  struct sdips_i2s_config*                 p_config_st                                                ,
  struct sdips_i2s_command*                p_cmd_st                                                   
) {
  uint16_t                m_idx_16b                                                                   ; // Index for write data
  //1-------------------------------------------------------------------------------------------------
  // Loop to transmit number of samples 
  //1-------------------------------------------------------------------------------------------------
  for (m_idx_16b = 0; m_idx_16b < p_cmd_st->num_samples_8b; m_idx_16b += 1) {
    write_reg(SDIPS_I2S_TX_FIFO,p_cmd_st->data_2d_32b[m_idx_16b]);
  }
}


//0---------------------------------------------------------------------------------------------------
// receive_sample :This method is used to receive sampler from receiver 
//0---------------------------------------------------------------------------------------------------
//             parameter :No parameter 
//0---------------------------------------------------------------------------------------------------
void receive_sample (
) {
  uint32_t                m_data_32b                                                                  ; // Data register 
  m_data_32b                 = read_reg(SDIPS_I2S_RX_FIFO);
}


//0---------------------------------------------------------------------------------------------------
// print_command :This method prints command 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//              p_cmd_st :Pointer to command object 
//0---------------------------------------------------------------------------------------------------
void print_command (
  struct sdips_i2s_config*                 p_config_st                                                ,
  struct sdips_i2s_command*                p_cmd_st                                                   
) {
  uint16_t                m_idx_16b                                                                   ; // Index for write data
  //1-------------------------------------------------------------------------------------------------
  // Loop to transmit number of samples 
  //1-------------------------------------------------------------------------------------------------
  for (m_idx_16b = 0; m_idx_16b < p_cmd_st->num_samples_8b; m_idx_16b += 1) {
    #ifdef SMARTDV_INFO
    printf("INFO : print_command() :: Index %0d Data 0x%x\n",p_cmd_st->data_2d_32b[m_idx_16b]);
    #endif
  }
}


//0---------------------------------------------------------------------------------------------------
// write_tx_config_csr :This method is used to write This is transmit configuration register 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void write_tx_config_csr (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  uint32_t                m_data_28b                                                                  ; // Register value to write
  m_data_28b                   = 0;
  m_data_28b                   = p_config_st->tx_enable_b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls number of transmit channels, 0 means 1 channel 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = (p_config_st->tx_num_channels_5b & 0x1F) << 1 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls transmit channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 
  // 8 bits, 63 means 64 bits. 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = (p_config_st->tx_channel_width_6b & 0x3F) << 6 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls transmit data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 
  // 8 bits, 63 means 64 bits. 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = (p_config_st->tx_data_width_6b & 0x3F) << 12 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls bit value to use in padding 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = p_config_st->tx_padding_b << 18 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls protocol mode 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = (p_config_st->tx_proto_mode_2b & 0x3) << 19 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls bit ordering 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = p_config_st->tx_bit_order_b << 21 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls justification, valid only in I2S mode 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = p_config_st->tx_justification_b << 22 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls WS to WD delay in SCK 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = (p_config_st->tx_sd_delay_3b & 0x7) << 23 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls transmit clock mode 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = p_config_st->tx_clk_mode_b << 26 | m_data_28b;
  //1-------------------------------------------------------------------------------------------------
  // Set Transmit clock polarity 
  //1-------------------------------------------------------------------------------------------------
  m_data_28b                     = p_config_st->tx_clk_pol_b << 27 | m_data_28b;
  write_reg(SDIPS_I2S_TX_CONFIG,m_data_28b);
}


//0---------------------------------------------------------------------------------------------------
// write_rx_config_csr :This method is used to write This is receive configuration register 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void write_rx_config_csr (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  uint32_t                m_data_27b                                                                  ; // Register value to write
  m_data_27b                   = 0;
  m_data_27b                   = p_config_st->rx_enable_b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls number of receive channels, 0 means 1 channel 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = (p_config_st->rx_num_channels_5b & 0x1F) << 1 | m_data_27b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls receive channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 
  // 8 bits, 63 means 64 bits. 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = (p_config_st->rx_channel_width_6b & 0x3F) << 6 | m_data_27b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls receive data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 
  // bits, 63 means 64 bits. 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = (p_config_st->rx_data_width_6b & 0x3F) << 12 | m_data_27b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls protocol mode 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = (p_config_st->rx_proto_mode_2b & 0x3) << 18 | m_data_27b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls bit ordering 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = p_config_st->rx_bit_order_b << 20 | m_data_27b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls justification, valid only in I2S mode 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = p_config_st->rx_justification_b << 21 | m_data_27b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls WS to WD delay in SCK 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = (p_config_st->rx_sd_delay_3b & 0x7) << 22 | m_data_27b;
  //1-------------------------------------------------------------------------------------------------
  // Set Controls receive clock mode 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = p_config_st->rx_clk_mode_b << 25 | m_data_27b;
  //1-------------------------------------------------------------------------------------------------
  // Set Receive clock polarity 
  //1-------------------------------------------------------------------------------------------------
  m_data_27b                     = p_config_st->rx_clk_pol_b << 26 | m_data_27b;
  write_reg(SDIPS_I2S_RX_CONFIG,m_data_27b);
}


//0---------------------------------------------------------------------------------------------------
// write_tx_prescaler_csr :This method is used to write This is transmit clock divider 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void write_tx_prescaler_csr (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  uint32_t                m_data_24b                                                                  ; // Register value to write
  m_data_24b                   = 0;
  m_data_24b                   = p_config_st->tx_mclk_div_16b;
  //1-------------------------------------------------------------------------------------------------
  // Set This is I2S clock divider, it divides o_i2s_tx_mck to generate o_i2s_tx_sck 
  //1-------------------------------------------------------------------------------------------------
  m_data_24b                     = (p_config_st->tx_i2s_div_8b & 0xFF) << 16 | m_data_24b;
  write_reg(SDIPS_I2S_TX_PRESCALER,m_data_24b);
}


//0---------------------------------------------------------------------------------------------------
// write_rx_prescaler_csr :This method is used to write This is receive clock divider 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void write_rx_prescaler_csr (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  uint32_t                m_data_24b                                                                  ; // Register value to write
  m_data_24b                   = 0;
  m_data_24b                   = p_config_st->rx_mclk_div_16b;
  //1-------------------------------------------------------------------------------------------------
  // Set This is I2S clock divider, it divides o_i2s_rx_mck to generate o_i2s_rx_sck 
  //1-------------------------------------------------------------------------------------------------
  m_data_24b                     = (p_config_st->rx_i2s_div_8b & 0xFF) << 16 | m_data_24b;
  write_reg(SDIPS_I2S_RX_PRESCALER,m_data_24b);
}


//0---------------------------------------------------------------------------------------------------
// write_irq_enable_csr :This method is used to write This register is IRQ enable register, 
//                       used to enable interrupts 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void write_irq_enable_csr (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  uint16_t                m_data_11b                                                                  ; // Register value to write
  m_data_11b                   = 0;
  m_data_11b                   = p_config_st->irq_rx_ready_b;
  //1-------------------------------------------------------------------------------------------------
  // Set Transmit FIFO threshold reached on read 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_tx_fifo_tt_b << 1 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Transmit FIFO underrun 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_tx_fifo_ur_b << 2 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Transmit FIFO overrun 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_tx_fifo_or_b << 3 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Transmit FIFO full 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_tx_fifo_full_b << 4 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Transmit FIFO empty 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_tx_fifo_empty_b << 5 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Receive FIFO threshold reached on write 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_rx_fifo_tt_b << 6 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Receive FIFO underrun 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_rx_fifo_ur_b << 7 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Receive FIFO overrun 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_rx_fifo_or_b << 8 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Receive FIFO full 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_rx_fifo_full_b << 9 | m_data_11b;
  //1-------------------------------------------------------------------------------------------------
  // Set Receive FIFO empty 
  //1-------------------------------------------------------------------------------------------------
  m_data_11b                     = p_config_st->irq_rx_fifo_empty_b << 10 | m_data_11b;
  write_reg(SDIPS_I2S_IRQ_ENABLE,m_data_11b);
}


//0---------------------------------------------------------------------------------------------------
// write_fifo_flush_csr :This method is used to write Register to flush FIFO's 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void write_fifo_flush_csr (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  uint8_t                 m_data_2b                                                                   ; // Register value to write
  m_data_2b                    = 0;
  m_data_2b                    = p_config_st->tx_fifo_flush_b;
  //1-------------------------------------------------------------------------------------------------
  // Set Receive Data FIFO flush 
  //1-------------------------------------------------------------------------------------------------
  m_data_2b                      = p_config_st->rx_fifo_flush_b << 1 | m_data_2b;
  write_reg(SDIPS_I2S_FIFO_FLUSH,m_data_2b);
}


//0---------------------------------------------------------------------------------------------------
// write_fifo_threshold_csr :This method is used to write Register to control threshold for 
//                           interrupt 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void write_fifo_threshold_csr (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  uint16_t                m_data_16b                                                                  ; // Register value to write
  m_data_16b                   = 0;
  m_data_16b                   = p_config_st->tx_fifo_threshold_8b;
  //1-------------------------------------------------------------------------------------------------
  // Set Receive FIFO threshold for write 
  //1-------------------------------------------------------------------------------------------------
  m_data_16b                     = (p_config_st->rx_fifo_threshold_8b & 0xFF) << 8 | m_data_16b;
  write_reg(SDIPS_I2S_FIFO_THRESHOLD,m_data_16b);
}


//0---------------------------------------------------------------------------------------------------
// write_soc_timeout_csr :This method is used to write SOC timeout register 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void write_soc_timeout_csr (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  uint8_t                 m_data_8b                                                                   ; // Register value to write
  m_data_8b                    = 0;
  m_data_8b                    = p_config_st->soc_timeout_8b;
  write_reg(SDIPS_I2S_SOC_TIMEOUT,m_data_8b);
}


//0---------------------------------------------------------------------------------------------------
// print_config :This method is used to print all config registers 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void print_config (
  struct sdips_i2s_config*                 p_config_st                                                
) {
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ------------------------------TX CONFIG Register----------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_ENABLE                          :: 0x%x\n",
    p_config_st->tx_enable_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_NUM_CHANNELS                    :: 0x%x\n",
    p_config_st->tx_num_channels_5b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_CHANNEL_WIDTH                   :: 0x%x\n",
    p_config_st->tx_channel_width_6b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_DATA_WIDTH                      :: 0x%x\n",
    p_config_st->tx_data_width_6b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_PADDING                         :: 0x%x\n",
    p_config_st->tx_padding_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_PROTO_MODE                      :: 0x%x\n",
    p_config_st->tx_proto_mode_2b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_BIT_ORDER                       :: 0x%x\n",
    p_config_st->tx_bit_order_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_JUSTIFICATION                   :: 0x%x\n",
    p_config_st->tx_justification_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_SD_DELAY                        :: 0x%x\n",
    p_config_st->tx_sd_delay_3b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_CLK_MODE                        :: 0x%x\n",
    p_config_st->tx_clk_mode_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_CLK_POL                         :: 0x%x\n",
    p_config_st->tx_clk_pol_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ------------------------------RX CONFIG Register----------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_ENABLE                          :: 0x%x\n",
    p_config_st->rx_enable_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_NUM_CHANNELS                    :: 0x%x\n",
    p_config_st->rx_num_channels_5b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_CHANNEL_WIDTH                   :: 0x%x\n",
    p_config_st->rx_channel_width_6b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_DATA_WIDTH                      :: 0x%x\n",
    p_config_st->rx_data_width_6b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_PROTO_MODE                      :: 0x%x\n",
    p_config_st->rx_proto_mode_2b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_BIT_ORDER                       :: 0x%x\n",
    p_config_st->rx_bit_order_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_JUSTIFICATION                   :: 0x%x\n",
    p_config_st->rx_justification_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_SD_DELAY                        :: 0x%x\n",
    p_config_st->rx_sd_delay_3b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_CLK_MODE                        :: 0x%x\n",
    p_config_st->rx_clk_mode_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_CLK_POL                         :: 0x%x\n",
    p_config_st->rx_clk_pol_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ------------------------------TX PRESCALER Register-------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_MCLK_DIV                        :: 0x%x\n",
    p_config_st->tx_mclk_div_16b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_I2S_DIV                         :: 0x%x\n",
    p_config_st->tx_i2s_div_8b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ------------------------------RX PRESCALER Register-------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_MCLK_DIV                        :: 0x%x\n",
    p_config_st->rx_mclk_div_16b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_I2S_DIV                         :: 0x%x\n",
    p_config_st->rx_i2s_div_8b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ------------------------------IRQ ENABLE Register---------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_RX_READY                       :: 0x%x\n",
    p_config_st->irq_rx_ready_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_TX_FIFO_TT                     :: 0x%x\n",
    p_config_st->irq_tx_fifo_tt_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_TX_FIFO_UR                     :: 0x%x\n",
    p_config_st->irq_tx_fifo_ur_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_TX_FIFO_OR                     :: 0x%x\n",
    p_config_st->irq_tx_fifo_or_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_TX_FIFO_FULL                   :: 0x%x\n",
    p_config_st->irq_tx_fifo_full_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_TX_FIFO_EMPTY                  :: 0x%x\n",
    p_config_st->irq_tx_fifo_empty_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_RX_FIFO_TT                     :: 0x%x\n",
    p_config_st->irq_rx_fifo_tt_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_RX_FIFO_UR                     :: 0x%x\n",
    p_config_st->irq_rx_fifo_ur_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_RX_FIFO_OR                     :: 0x%x\n",
    p_config_st->irq_rx_fifo_or_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_RX_FIFO_FULL                   :: 0x%x\n",
    p_config_st->irq_rx_fifo_full_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: IRQ_RX_FIFO_EMPTY                  :: 0x%x\n",
    p_config_st->irq_rx_fifo_empty_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ------------------------------FIFO FLUSH Register---------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_FIFO_FLUSH                      :: 0x%x\n",
    p_config_st->tx_fifo_flush_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_FIFO_FLUSH                      :: 0x%x\n",
    p_config_st->rx_fifo_flush_b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ------------------------------FIFO THRESHOLD Register-----------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: TX_FIFO_THRESHOLD                  :: 0x%x\n",
    p_config_st->tx_fifo_threshold_8b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: RX_FIFO_THRESHOLD                  :: 0x%x\n",
    p_config_st->rx_fifo_threshold_8b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ------------------------------SOC TIMEOUT Register--------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: SOC_TIMEOUT                        :: 0x%x\n",
    p_config_st->soc_timeout_8b);
  #endif
  #ifdef SMARTDV_INFO
  printf("INFO : print_config() :: ----------------------------------------------------------------------------------\n");
  #endif
}


//0---------------------------------------------------------------------------------------------------
// irq_handler :This method is used to handle the interupts generated by core 
//0---------------------------------------------------------------------------------------------------
//              p_cmd_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
void irq_handler (
  struct sdips_i2s_command*                p_cmd_st                                                   
) {
  uint8_t                 m_bit_b                                                                     ; // Variable to check a single IRQ assertion
  sdips_i2s_irq_status_32b     = read_reg(SDIPS_I2S_IRQ_STATUS);
  write_reg(SDIPS_I2S_IRQ_STATUS,sdips_i2s_irq_status_32b);
  //1-------------------------------------------------------------------------------------------------
  // get the value of bit from  to 0 
  //1-------------------------------------------------------------------------------------------------
  m_bit_b                        = (sdips_i2s_irq_status_32b & 0x1);
  //1-------------------------------------------------------------------------------------------------
  // Check if interupt asserted 
  //1-------------------------------------------------------------------------------------------------
  if (m_bit_b == 1) {
    receive_sample ();
  }
}




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
