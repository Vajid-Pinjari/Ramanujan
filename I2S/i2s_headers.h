#ifndef _MFR_IIS_H 
#define _MFR_IIS_H 

/* I2S Register Information header file
 *written by reference of rockchips reference code 
 *Ramanujan's reg details not came yet*/

/* I2S REGS */
#define I2S_TXCR	(0x0000)      //transmit operation control register
#define I2S_RXCR	(0x0004)      //receive operation control register
#define I2S_CKR		(0x0008)      //clock generation register
#define I2S_FIFOLR	(0x000c)      //FIFO level register
#define I2S_DMACR	(0x0010)      //DMA control register
#define I2S_INTCR	(0x0014)      //interrupt control register
#define I2S_INTSR	(0x0018)      //interrupt status register
#define I2S_XFER	(0x001c)      //Transfer Start Register
#define I2S_CLR		(0x0020)      //SCLK domain logic clear Register
#define I2S_TXDR	(0x0024)      //Transimt FIFO Data Register
#define I2S_RXDR	(0x0028)      //Receive FIFO Data Register


/*
 * TXCR
 * transmit operation control register
*/
#define I2S_TXCR_RCNT_SHIFT	17                             //right justified counter                         
#define I2S_TXCR_RCNT_MASK	(0x3f << I2S_TXCR_RCNT_SHIFT)  //right justified counter mask
#define I2S_TXCR_CSR_SHIFT	15                             //channel select register
#define I2S_TXCR_CSR(x)		(x << I2S_TXCR_CSR_SHIFT)      //select channel
#define I2S_TXCR_CSR_MASK	(3 << I2S_TXCR_CSR_SHIFT)      //mask channel select register
#define I2S_TXCR_HWT		BIT(14)                        //half word select bit (if.16 bit data)
#define I2S_TXCR_SJM_SHIFT	12                             //store justified mode
#define I2S_TXCR_SJM_R		(0 << I2S_TXCR_SJM_SHIFT)      //for right justified
#define I2S_TXCR_SJM_L		(1 << I2S_TXCR_SJM_SHIFT)      //for left justified
#define I2S_TXCR_FBM_SHIFT	11                             //first bit mode
#define I2S_TXCR_FBM_MSB	(0 << I2S_TXCR_FBM_SHIFT)      //xfer MSB first
#define I2S_TXCR_FBM_LSB	(1 << I2S_TXCR_FBM_SHIFT)      //xfer LSB first
#define I2S_TXCR_IBM_SHIFT	9                              //I2S bus mode
#define I2S_TXCR_IBM_NORMAL	(0 << I2S_TXCR_IBM_SHIFT)      //normal
#define I2S_TXCR_IBM_LSJM	(1 << I2S_TXCR_IBM_SHIFT)      //left justified
#define I2S_TXCR_IBM_RSJM	(2 << I2S_TXCR_IBM_SHIFT)      //right justified
#define I2S_TXCR_IBM_MASK	(3 << I2S_TXCR_IBM_SHIFT)      //reserved/MASK
#define I2S_TXCR_PBM_SHIFT	7                              //PCM Mode
#define I2S_TXCR_PBM_MODE(x)	(x << I2S_TXCR_PBM_SHIFT)  
#define I2S_TXCR_PBM_MASK	(3 << I2S_TXCR_PBM_SHIFT)  
#define I2S_TXCR_TFS_SHIFT	5                              //Transfer format select
#define I2S_TXCR_TFS_I2S	(0 << I2S_TXCR_TFS_SHIFT)      //I2S format
#define I2S_TXCR_TFS_PCM	(1 << I2S_TXCR_TFS_SHIFT)      //PCM Format
#define I2S_TXCR_TFS_MASK	(1 << I2S_TXCR_TFS_SHIFT)      //Mask format
#define I2S_TXCR_VDW_SHIFT	0                              //Valid Data width
#define I2S_TXCR_VDW(x)		((x - 1) << I2S_TXCR_VDW_SHIFT) 
#define I2S_TXCR_VDW_MASK	(0x1f << I2S_TXCR_VDW_SHIFT)

/* channel select*/
#define I2S_CSR_SHIFT	15                 //Channels
#define I2S_CHN_2	(0 << I2S_CSR_SHIFT)   //channel 0 enable
#define I2S_CHN_4	(1 << I2S_CSR_SHIFT)   //channel 0 & 1 enable
#define I2S_CHN_6	(2 << I2S_CSR_SHIFT)   //channel 0, 1, 2 enable
#define I2S_CHN_8	(3 << I2S_CSR_SHIFT)   //channel 0,1,2&3 enable

/*
 * RXCR
 * receive operation control register
*/
#define I2S_RXCR_CSR_SHIFT	15                               //reserved ?//                        
#define I2S_RXCR_CSR(x)		(x << I2S_RXCR_CSR_SHIFT)  
#define I2S_RXCR_CSR_MASK	(3 << I2S_RXCR_CSR_SHIFT)  
#define I2S_RXCR_HWT		BIT(14)                          //Halfword word transform
#define I2S_RXCR_SJM_SHIFT	12                               //store justified mode
#define I2S_RXCR_SJM_R		(0 << I2S_RXCR_SJM_SHIFT)        //right justified mode
#define I2S_RXCR_SJM_L		(1 << I2S_RXCR_SJM_SHIFT)        //left justified mode
#define I2S_RXCR_FBM_SHIFT	11                               //first bit mode
#define I2S_RXCR_FBM_MSB	(0 << I2S_RXCR_FBM_SHIFT)        //MSB first
#define I2S_RXCR_FBM_LSB	(1 << I2S_RXCR_FBM_SHIFT)        //LSB first
#define I2S_RXCR_IBM_SHIFT	9                                //I2S Bus mode
#define I2S_RXCR_IBM_NORMAL	(0 << I2S_RXCR_IBM_SHIFT)        //mode 0
#define I2S_RXCR_IBM_LSJM	(1 << I2S_RXCR_IBM_SHIFT)        //mode 1
#define I2S_RXCR_IBM_RSJM	(2 << I2S_RXCR_IBM_SHIFT)        //mode 2
#define I2S_RXCR_IBM_MASK	(3 << I2S_RXCR_IBM_SHIFT)        //mode 3
#define I2S_RXCR_PBM_SHIFT	7                                //PCM Bus mode
#define I2S_RXCR_PBM_MODE(x)	(x << I2S_RXCR_PBM_SHIFT)
#define I2S_RXCR_PBM_MASK	(3 << I2S_RXCR_PBM_SHIFT)         
#define I2S_RXCR_TFS_SHIFT	5                                //Transfer format select i2s/pcm
#define I2S_RXCR_TFS_I2S	(0 << I2S_RXCR_TFS_SHIFT)       //i2s
#define I2S_RXCR_TFS_PCM	(1 << I2S_RXCR_TFS_SHIFT)       //pcm
#define I2S_RXCR_TFS_MASK	(1 << I2S_RXCR_TFS_SHIFT)       //mask format
#define I2S_RXCR_VDW_SHIFT	0                               //valid data width
#define I2S_RXCR_VDW(x)		((x - 1) << I2S_RXCR_VDW_SHIFT) 
#define I2S_RXCR_VDW_MASK	(0x1f << I2S_RXCR_VDW_SHIFT)

/*
 * CKR
 * clock generation register
*/
#define I2S_CKR_TRCM_SHIFT	28                             //reserved ?
#define I2S_CKR_TRCM(x)	(x << I2S_CKR_TRCM_SHIFT)
#define I2S_CKR_TRCM_TXRX	(0 << I2S_CKR_TRCM_SHIFT)
#define I2S_CKR_TRCM_TXONLY	(1 << I2S_CKR_TRCM_SHIFT)
#define I2S_CKR_TRCM_RXONLY	(2 << I2S_CKR_TRCM_SHIFT)
#define I2S_CKR_TRCM_MASK	(3 << I2S_CKR_TRCM_SHIFT)
#define I2S_CKR_MSS_SHIFT	27                            //Master/slave mode select
#define I2S_CKR_MSS_MASTER	(0 << I2S_CKR_MSS_SHIFT)      //master mode
#define I2S_CKR_MSS_SLAVE	(1 << I2S_CKR_MSS_SHIFT)      //slave mode
#define I2S_CKR_MSS_MASK	(1 << I2S_CKR_MSS_SHIFT)      
#define I2S_CKR_CKP_SHIFT	26                            //sclk polarity
#define I2S_CKR_CKP_NORMAL	(0 << I2S_CKR_CKP_SHIFT)      
#define I2S_CKR_CKP_INVERTED	(1 << I2S_CKR_CKP_SHIFT)
#define I2S_CKR_CKP_MASK	(1 << I2S_CKR_CKP_SHIFT)
#define I2S_CKR_RLP_SHIFT	25                            //receive lrck polarity
#define I2S_CKR_RLP_NORMAL	(0 << I2S_CKR_RLP_SHIFT)      
#define I2S_CKR_RLP_INVERTED	(1 << I2S_CKR_RLP_SHIFT)
#define I2S_CKR_RLP_MASK	(1 << I2S_CKR_RLP_SHIFT)
#define I2S_CKR_TLP_SHIFT	24                           //transmit lrck polarity
#define I2S_CKR_TLP_NORMAL	(0 << I2S_CKR_TLP_SHIFT)
#define I2S_CKR_TLP_INVERTED	(1 << I2S_CKR_TLP_SHIFT)
#define I2S_CKR_TLP_MASK	(1 << I2S_CKR_TLP_SHIFT)
#define I2S_CKR_MDIV_SHIFT	16                           //mclk devider
#define I2S_CKR_MDIV(x)		((x - 1) << I2S_CKR_MDIV_SHIFT)
#define I2S_CKR_MDIV_MASK	(0xff << I2S_CKR_MDIV_SHIFT)
#define I2S_CKR_RSD_SHIFT	8                            //receive sclk devider
#define I2S_CKR_RSD(x)		((x - 1) << I2S_CKR_RSD_SHIFT)
#define I2S_CKR_RSD_MASK	(0xff << I2S_CKR_RSD_SHIFT)
#define I2S_CKR_TSD_SHIFT	0                            //transmit sclk devider
#define I2S_CKR_TSD(x)		((x - 1) << I2S_CKR_TSD_SHIFT)
#define I2S_CKR_TSD_MASK	(0xff << I2S_CKR_TSD_SHIFT)

 /* FIFOLR
 * FIFO level register 
 It tells in Rx fifo how many words are available to read  
 or in Tx how many slots are empty to write
 */

#define I2S_FIFOLR_RFL_SHIFT	24                               //receive FIFO level
#define I2S_FIFOLR_RFL_MASK	(0x3f << I2S_FIFOLR_RFL_SHIFT)       //RFL mask
#define I2S_FIFOLR_TFL3_SHIFT	18                               //Transmit FIFO level 3
#define I2S_FIFOLR_TFL3_MASK	(0x3f << I2S_FIFOLR_TFL3_SHIFT)  //mask TFL3
#define I2S_FIFOLR_TFL2_SHIFT	12                               //TFL2
#define I2S_FIFOLR_TFL2_MASK	(0x3f << I2S_FIFOLR_TFL2_SHIFT)  //mask TFL2
#define I2S_FIFOLR_TFL1_SHIFT	6                                //TFL1
#define I2S_FIFOLR_TFL1_MASK	(0x3f << I2S_FIFOLR_TFL1_SHIFT)  //TFL1 mask
#define I2S_FIFOLR_TFL0_SHIFT	0                                //TFL0
#define I2S_FIFOLR_TFL0_MASK	(0x3f << I2S_FIFOLR_TFL0_SHIFT)  //Mask TFL0

 /*
 * DMACR
 * DMA control register
 */

#define I2S_DMACR_RDE_SHIFT	24                                //Receive DMA enable
#define I2S_DMACR_RDE_DISABLE	(0 << I2S_DMACR_RDE_SHIFT)    //Disable
#define I2S_DMACR_RDE_ENABLE	(1 << I2S_DMACR_RDE_SHIFT)    //Enable
#define I2S_DMACR_RDL_SHIFT	16                                //Receive DATA level
#define I2S_DMACR_RDL(x)	((x - 1) << I2S_DMACR_RDL_SHIFT)  //
#define I2S_DMACR_RDL_MASK	(0x1f << I2S_DMACR_RDL_SHIFT)
#define I2S_DMACR_TDE_SHIFT	8                                 //transmit DMA Enable
#define I2S_DMACR_TDE_DISABLE	(0 << I2S_DMACR_TDE_SHIFT)    //disable
#define I2S_DMACR_TDE_ENABLE	(1 << I2S_DMACR_TDE_SHIFT)    //enable
#define I2S_DMACR_TDL_SHIFT	0                                 //Transmit Data Level
#define I2S_DMACR_TDL(x)	((x) << I2S_DMACR_TDL_SHIFT)
#define I2S_DMACR_TDL_MASK	(0x1f << I2S_DMACR_TDL_SHIFT)

/*
 * INTCR
 * interrupt control register
*/

#define I2S_INTCR_RFT_SHIFT	20                                 //Receive FIFO Threshold
#define I2S_INTCR_RFT(x)	((x - 1) << I2S_INTCR_RFT_SHIFT)   //
#define I2S_INTCR_RXOIC		BIT(18)                            //RX overrun interrupt clear
#define I2S_INTCR_RXOIE_SHIFT	17                             //RX overrun interrupt enable
#define I2S_INTCR_RXOIE_DISABLE	(0 << I2S_INTCR_RXOIE_SHIFT)   //Disable
#define I2S_INTCR_RXOIE_ENABLE	(1 << I2S_INTCR_RXOIE_SHIFT)   //Enable
#define I2S_INTCR_RXFIE_SHIFT	16                             //RX full interrupt enable
#define I2S_INTCR_RXFIE_DISABLE	(0 << I2S_INTCR_RXFIE_SHIFT)   //Disable
#define I2S_INTCR_RXFIE_ENABLE	(1 << I2S_INTCR_RXFIE_SHIFT)   //Enable
#define I2S_INTCR_TFT_SHIFT	4                                  //Transmit FIFO threshould
#define I2S_INTCR_TFT(x)	((x - 1) << I2S_INTCR_TFT_SHIFT)   //Set threshold (depend on CSR)
#define I2S_INTCR_TFT_MASK	(0x1f << I2S_INTCR_TFT_SHIFT)      //Mask threshold
#define I2S_INTCR_TXUIC		BIT(2)                             //TX underrun interrupt clear
#define I2S_INTCR_TXUIE_SHIFT	1                              //TX underrun interrupt enable
#define I2S_INTCR_TXUIE_DISABLE	(0 << I2S_INTCR_TXUIE_SHIFT)   //disable
#define I2S_INTCR_TXUIE_ENABLE	(1 << I2S_INTCR_TXUIE_SHIFT)   //enable

/*
 * INTSR
 * interrupt status register
*/

#define I2S_INTSR_TXEIE_SHIFT	0                              //TX empty interrupt
#define I2S_INTSR_TXEIE_DISABLE	(0 << I2S_INTSR_TXEIE_SHIFT)   //inactive
#define I2S_INTSR_TXEIE_ENABLE	(1 << I2S_INTSR_TXEIE_SHIFT)   //active
#define I2S_INTSR_RXOI_SHIFT	17                             //RX overrun interrupt
#define I2S_INTSR_RXOI_INA	(0 << I2S_INTSR_RXOI_SHIFT)        //inactive
#define I2S_INTSR_RXOI_ACT	(1 << I2S_INTSR_RXOI_SHIFT)        //active
#define I2S_INTSR_RXFI_SHIFT	16                             //RX full interrupt
#define I2S_INTSR_RXFI_INA	(0 << I2S_INTSR_RXFI_SHIFT)        //Inactive
#define I2S_INTSR_RXFI_ACT	(1 << I2S_INTSR_RXFI_SHIFT)        //active
#define I2S_INTSR_TXUI_SHIFT	1                              //TX underrun interrupt
#define I2S_INTSR_TXUI_INA	(0 << I2S_INTSR_TXUI_SHIFT)        //inactive
#define I2S_INTSR_TXUI_ACT	(1 << I2S_INTSR_TXUI_SHIFT)        //active
#define I2S_INTSR_TXEI_SHIFT	0                              //TX empty interrupt
#define I2S_INTSR_TXEI_INA	(0 << I2S_INTSR_TXEI_SHIFT)        //Inactive
#define I2S_INTSR_TXEI_ACT	(1 << I2S_INTSR_TXEI_SHIFT)        //active

/*
 * XFER
 * Transfer start register
*/

#define I2S_XFER_RXS_SHIFT	1                                  //RX Transfer start bit
#define I2S_XFER_RXS_STOP	(0 << I2S_XFER_RXS_SHIFT)          //stop
#define I2S_XFER_RXS_START	(1 << I2S_XFER_RXS_SHIFT)          //start
#define I2S_XFER_TXS_SHIFT	0                                  //TX Transfer start bit
#define I2S_XFER_TXS_STOP	(0 << I2S_XFER_TXS_SHIFT)          //stop
#define I2S_XFER_TXS_START	(1 << I2S_XFER_TXS_SHIFT)          //start

/*
 * CLR
 * clear SCLK domain logic register
*/

#define I2S_CLR_RXC	BIT(1)  //RX logic clear
#define I2S_CLR_TXC	BIT(0)  //TX logic clear

/*
 * TXDR
 * Transimt FIFO data register, write only.
 */
#define I2S_TXDR_MASK	(0xff)

/*
 * RXDR
 * Receive FIFO data register, write only.
 */
#define I2S_RXDR_MASK	(0xff)

/* Clock divider id
*/ 
enum {
	MFR_DIV_MCLK = 0,
	MFR_DIV_BCLK,
};




/* io direction cfg register */
#define I2S_IO_DIRECTION_MASK	(7)
#define I2S_IO_8CH_OUT_2CH_IN	(0)
#define I2S_IO_6CH_OUT_4CH_IN	(4)
#define I2S_IO_4CH_OUT_6CH_IN	(6)
#define I2S_IO_2CH_OUT_8CH_IN	(7)

#endif /* _MFR_IIS_H */
