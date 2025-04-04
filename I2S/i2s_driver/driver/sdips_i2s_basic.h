#ifndef SDIPS_I2S_BASIC_H
#define SDIPS_I2S_BASIC_H
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
// Description    : This is C header file for sdips_i2s IP core 
// Generator      : IIP Compiler Version 1.1
//0---------------------------------------------------------------------------------------------------
struct sdips_i2s_command {
  uint32_t                data_2d_32b [32]                                                            ; // Holds the data to send/received
  uint8_t                 num_samples_8b                                                              ; // Holds number of data to send/received
};

struct sdips_i2s_config {
  uint8_t                 tx_enable_b                                                                 ; // Transmitter enable
  uint8_t                 tx_num_channels_5b                                                          ; // Controls number of transmit channels, 0 means 1 channel
  uint8_t                 tx_channel_width_6b                                                         ; // Controls transmit channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
  uint8_t                 tx_data_width_6b                                                            ; // Controls transmit data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
  uint8_t                 tx_padding_b                                                                ; // Controls bit value to use in padding
  uint8_t                 tx_proto_mode_2b                                                            ; // Controls protocol mode
  uint8_t                 tx_bit_order_b                                                              ; // Controls bit ordering
  uint8_t                 tx_justification_b                                                          ; // Controls justification, valid only in I2S mode
  uint8_t                 tx_sd_delay_3b                                                              ; // Controls WS to WD delay in SCK
  uint8_t                 tx_clk_mode_b                                                               ; // Controls transmit clock mode
  uint8_t                 tx_clk_pol_b                                                                ; // Transmit clock polarity
  uint8_t                 rx_enable_b                                                                 ; // Receiver enable
  uint8_t                 rx_num_channels_5b                                                          ; // Controls number of receive channels, 0 means 1 channel
  uint8_t                 rx_channel_width_6b                                                         ; // Controls receive channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
  uint8_t                 rx_data_width_6b                                                            ; // Controls receive data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
  uint8_t                 rx_proto_mode_2b                                                            ; // Controls protocol mode
  uint8_t                 rx_bit_order_b                                                              ; // Controls bit ordering
  uint8_t                 rx_justification_b                                                          ; // Controls justification, valid only in I2S mode
  uint8_t                 rx_sd_delay_3b                                                              ; // Controls WS to WD delay in SCK
  uint8_t                 rx_clk_mode_b                                                               ; // Controls receive clock mode
  uint8_t                 rx_clk_pol_b                                                                ; // Receive clock polarity
  uint16_t                tx_mclk_div_16b                                                             ; // This is Master clock divider, it divides i_clk to generate o_i2s_tx_mck
  uint8_t                 tx_i2s_div_8b                                                               ; // This is I2S clock divider, it divides o_i2s_tx_mck to generate o_i2s_tx_sck
  uint16_t                rx_mclk_div_16b                                                             ; // This is Master clock divider, it divides i_clk to generate o_i2s_rx_mck
  uint8_t                 rx_i2s_div_8b                                                               ; // This is I2S clock divider, it divides o_i2s_rx_mck to generate o_i2s_rx_sck
  uint8_t                 irq_rx_ready_b                                                              ; // Receive FIFO has data
  uint8_t                 irq_tx_fifo_tt_b                                                            ; // Transmit FIFO threshold reached on read
  uint8_t                 irq_tx_fifo_ur_b                                                            ; // Transmit FIFO underrun
  uint8_t                 irq_tx_fifo_or_b                                                            ; // Transmit FIFO overrun
  uint8_t                 irq_tx_fifo_full_b                                                          ; // Transmit FIFO full
  uint8_t                 irq_tx_fifo_empty_b                                                         ; // Transmit FIFO empty
  uint8_t                 irq_rx_fifo_tt_b                                                            ; // Receive FIFO threshold reached on write
  uint8_t                 irq_rx_fifo_ur_b                                                            ; // Receive FIFO underrun
  uint8_t                 irq_rx_fifo_or_b                                                            ; // Receive FIFO overrun                 
  uint8_t                 irq_rx_fifo_full_b                                                          ; // Receive FIFO full
  uint8_t                 irq_rx_fifo_empty_b                                                         ; // Receive FIFO empty
  uint8_t                 tx_fifo_flush_b                                                             ; // Transmit Data FIFO flush
  uint8_t                 rx_fifo_flush_b                                                             ; // Receive Data FIFO flush
  uint8_t                 tx_fifo_threshold_8b                                                        ; // Transmit FIFO threshold for read
  uint8_t                 rx_fifo_threshold_8b                                                        ; // Receive FIFO threshold for write
  uint8_t                 soc_timeout_8b                                                              ; // SOC timeout register
};

uint32_t                sdips_i2s_irq_status_32b                                                    ; // Variable to check a single IRQ assertion


//0---------------------------------------------------------------------------------------------------
// write_reg :This is for register write 
//0---------------------------------------------------------------------------------------------------
//             p_addr_9b :Register address to write 
//            p_data_32b :Register data to write 
//0---------------------------------------------------------------------------------------------------
extern void write_reg (
  uint16_t                  p_addr_9b                                                                 ,
  uint32_t                  p_data_32b                                                                
);


//0---------------------------------------------------------------------------------------------------
// read_reg :This is for register read 
//0---------------------------------------------------------------------------------------------------
//               returns :Return the read data 
//             p_addr_9b :Register address to read 
//0---------------------------------------------------------------------------------------------------
extern uint32_t read_reg (
  uint16_t                  p_addr_9b                                                                 
);


//0---------------------------------------------------------------------------------------------------
// write_byte_reg :This is for Write Byte register 
//0---------------------------------------------------------------------------------------------------
//             p_addr_9b :Register address to write 
//             p_data_8b :Register data to write 
//0---------------------------------------------------------------------------------------------------
extern void write_byte_reg (
  uint16_t                  p_addr_9b                                                                 ,
  uint8_t                   p_data_8b                                                                 
);


//0---------------------------------------------------------------------------------------------------
// write_memory :This is for memory write 
//0---------------------------------------------------------------------------------------------------
//            p_addr_32b :Memory address to write 
//            p_data_32b :Memory data to write 
//0---------------------------------------------------------------------------------------------------
extern void write_memory (
  uint32_t                  p_addr_32b                                                                ,
  uint32_t                  p_data_32b                                                                
);


//0---------------------------------------------------------------------------------------------------
// read_memory :This is for memory read 
//0---------------------------------------------------------------------------------------------------
//               returns :Return the read data 
//            p_addr_32b :Memory address to read 
//0---------------------------------------------------------------------------------------------------
extern uint32_t read_memory (
  uint32_t                  p_addr_32b                                                                
);


//0---------------------------------------------------------------------------------------------------
// mask_irq :This method is used to mask the IRQ 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//            p_mask_21b :Interrupts to mask 
//0---------------------------------------------------------------------------------------------------
extern void mask_irq (
  struct sdips_i2s_config*                 p_config_st                                                ,
  uint32_t                  p_mask_21b                                                                
);


//0---------------------------------------------------------------------------------------------------
// unmask_irq :This method is used to unmask the IRQ 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//            p_mask_21b :Interrupts to mask 
//0---------------------------------------------------------------------------------------------------
extern void unmask_irq (
  struct sdips_i2s_config*                 p_config_st                                                ,
  uint32_t                  p_mask_21b                                                                
);


//0---------------------------------------------------------------------------------------------------
// init_chip :This method is used to initialize the I2S core 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void init_chip (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// enable_chip :This method is used to enable/disable chip 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//            p_enable_b :1 means enable, 0 means disable 
//0---------------------------------------------------------------------------------------------------
extern void enable_chip (
  struct sdips_i2s_config*                 p_config_st                                                ,
  uint8_t                   p_enable_b                                                                
);


//0---------------------------------------------------------------------------------------------------
// reset_chip :This method is used to reset the chip 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void reset_chip (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// enable_data_irq :This method is used to enable Data IRQ's 
//0---------------------------------------------------------------------------------------------------
//             parameter :No parameter 
//0---------------------------------------------------------------------------------------------------
extern void enable_data_irq (
);


//0---------------------------------------------------------------------------------------------------
// send_sample :This method is used to transmit audio sample from transmitter 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//              p_cmd_st :Pointer to command object 
//0---------------------------------------------------------------------------------------------------
extern void send_sample (
  struct sdips_i2s_config*                 p_config_st                                                ,
  struct sdips_i2s_command*                p_cmd_st                                                   
);


//0---------------------------------------------------------------------------------------------------
// receive_sample :This method is used to receive sampler from receiver 
//0---------------------------------------------------------------------------------------------------
//             parameter :No parameter 
//0---------------------------------------------------------------------------------------------------
extern void receive_sample (
);


//0---------------------------------------------------------------------------------------------------
// print_command :This method prints command 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//              p_cmd_st :Pointer to command object 
//0---------------------------------------------------------------------------------------------------
extern void print_command (
  struct sdips_i2s_config*                 p_config_st                                                ,
  struct sdips_i2s_command*                p_cmd_st                                                   
);


//0---------------------------------------------------------------------------------------------------
// write_tx_config_csr :This method is used to write This is transmit configuration register 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void write_tx_config_csr (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// write_rx_config_csr :This method is used to write This is receive configuration register 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void write_rx_config_csr (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// write_tx_prescaler_csr :This method is used to write This is transmit clock divider 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void write_tx_prescaler_csr (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// write_rx_prescaler_csr :This method is used to write This is receive clock divider 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void write_rx_prescaler_csr (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// write_irq_enable_csr :This method is used to write This register is IRQ enable register, 
//                       used to enable interrupts 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void write_irq_enable_csr (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// write_fifo_flush_csr :This method is used to write Register to flush FIFO's 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void write_fifo_flush_csr (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// write_fifo_threshold_csr :This method is used to write Register to control threshold for 
//                           interrupt 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void write_fifo_threshold_csr (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// write_soc_timeout_csr :This method is used to write SOC timeout register 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void write_soc_timeout_csr (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// print_config :This method is used to print all config registers 
//0---------------------------------------------------------------------------------------------------
//           p_config_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void print_config (
  struct sdips_i2s_config*                 p_config_st                                                
);


//0---------------------------------------------------------------------------------------------------
// irq_handler :This method is used to handle the interupts generated by core 
//0---------------------------------------------------------------------------------------------------
//              p_cmd_st :Pointer to config object 
//0---------------------------------------------------------------------------------------------------
extern void irq_handler (
  struct sdips_i2s_command*                p_cmd_st                                                   
);


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
