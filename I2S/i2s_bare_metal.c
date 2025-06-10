#define TX_CONFIG                       0x0       // This is transmit configuration register
#define RX_CONFIG                       0x4       // This is receive configuration register
#define TX_PRESCALER                    0x8       // This is transmit clock divider
#define RX_PRESCALER                    0xC       // This is receive clock divider
#define IRQ_ENABLE                      0x10      // This register is IRQ enable register, used to enable interrupts
#define IRQ_STATUS                      0x14      //IRQ status register
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
#define TX_ENABLE_MASK                  (1 << TX_ENABLE) 
#define TX_NUM_CHANNELS                 1         // Controls number of transmit channels, 0 means 1 channel
#define TX_NUM_CHANNELS_MASK            (0x1F << TX_NUM_CHANNELS)
#define TX_CHANNEL_WIDTH                6         // Controls transmit channel width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define TX_CHANNEL_WIDTH_MASK           (0x3F << TX_CHANNEL_WIDTH)
#define TX_DATA_WIDTH                   12        // Controls transmit data width in bits, default is 16 bits wide, 0 means 1 bit, 7 means 8 bits, 63 means 64 bits.
#define TX_DATA_WIDTH_MASK              (0x3F << TX_DATA_WIDTH)
#define TX_PADDING                      18        // Controls bit value to use in padding
#define TX_PADDING_MASK                 (0x1 << TX_PADDING)
#define TX_PROTO_MODE                   19        // Controls protocol mode
#define TX_PROTO_MODE_MASK              (0x3 << TX_PROTO_MODE)
#define TX_BIT_ORDER                    21        // Controls bit ordering
#define TX_BIT_ORDER_MASK               (0x1 << TX_BIT_ORDER)
#define TX_JUSTIFICATION                22        // Controls justification, valid only in I2S mode
#define TX_JUSTIFICATION_MASK           (0x1 << TX_JUSTIFICATION)
#define TX_SD_DELAY                     23        // Controls WS to WD delay in SCK
#define TX_SD_DELAY_MASK                (0x7 << TX_SD_DELAY)
#define TX_CLK_MODE                     26        // Controls transmit clock mode
#define TX_CLK_MODE_MASK                (0x1 << TX_CLK_MODE)
#define TX_CLK_POL                      27        // Transmit clock polarity
#define TX_CLK_POL_MASK                 (0x1 << TX_CLK_POL)
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR RX_CONFIG
//0-------------------------------------------------------------------------------
#define RX_ENABLE                       0         // Receiver enable
#define RX_ENABLE_MASK                  (0x1 << RX_ENABLE)
#define RX_NUM_CHANNELS                 1         // Controls number of receive channels, 0 means 1 channel
#define RX_NUM_CHANNELS_MASK            (0x1F << RX_NUM_CHANNELS)
#define RX_CHANNEL_WIDTH                6         // Controls receive channel width in bits
#define RX_CHANNEL_WIDTH_MASK           (0x3F << RX_CHANNEL_WIDTH)
#define RX_DATA_WIDTH                   12        // Controls receive data width in bits
#define RX_DATA_WIDTH_MASK              (0x3F << RX_DATA_WIDTH)
#define RX_PROTO_MODE                   18        // Controls protocol mode
#define RX_PROTO_MODE_MASK              (0x3 << RX_PROTO_MODE)
#define RX_BIT_ORDER                    20        // Controls bit ordering
#define RX_BIT_ORDER_MASK               (0x1 << RX_BIT_ORDER)
#define RX_JUSTIFICATION                21        // Controls justification
#define RX_JUSTIFICATION_MASK           (0x1 << RX_JUSTIFICATION)
#define RX_SD_DELAY                     22        // Controls WS to WD delay
#define RX_SD_DELAY_MASK                (0x7 << RX_SD_DELAY)
#define RX_CLK_MODE                     25        // Controls receive clock mode
#define RX_CLK_MODE_MASK                (0x1 << RX_CLK_MODE)
#define RX_CLK_POL                      26        // Receive clock polarity
#define RX_CLK_POL_MASK                 (0x1 << RX_CLK_POL)

//0-------------------------------------------------------------------------------
// Defines for subfields of CSR TX_PRESCALER
//0-------------------------------------------------------------------------------
#define TX_MCLK_DIV                     0         // Master clock divider (bits 0:15)
#define TX_MCLK_DIV_MASK                (0xFFFF << TX_MCLK_DIV)
#define TX_I2S_DIV                      16        // I2S clock divider (bits 16:23)
#define TX_I2S_DIV_MASK                 (0xFF << TX_I2S_DIV)
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR RX_PRESCALER
//0-------------------------------------------------------------------------------
#define RX_MCLK_DIV                     0         // Master clock divider (bits 0:15)
#define RX_MCLK_DIV_MASK                (0xFFFF << RX_MCLK_DIV)
#define RX_I2S_DIV                      16        // I2S clock divider (bits 16:23)
#define RX_I2S_DIV_MASK                 (0xFF << RX_I2S_DIV)
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR IRQ_ENABLE
//0-------------------------------------------------------------------------------
#define IRQ_RX_READY                    0         // Receive FIFO has data
#define IRQ_RX_READY_MASK               (0x1 << IRQ_RX_READY)
#define IRQ_TX_FIFO_TT                  1         // Transmit FIFO threshold reached
#define IRQ_TX_FIFO_TT_MASK             (0x1 << IRQ_TX_FIFO_TT)
#define IRQ_TX_FIFO_UR                  2         // Transmit FIFO underrun
#define IRQ_TX_FIFO_UR_MASK             (0x1 << IRQ_TX_FIFO_UR)
#define IRQ_TX_FIFO_OR                  3         // Transmit FIFO overrun
#define IRQ_TX_FIFO_OR_MASK             (0x1 << IRQ_TX_FIFO_OR)
#define IRQ_TX_FIFO_FULL                4         // Transmit FIFO full
#define IRQ_TX_FIFO_FULL_MASK           (0x1 << IRQ_TX_FIFO_FULL)
#define IRQ_TX_FIFO_EMPTY               5         // Transmit FIFO empty
#define IRQ_TX_FIFO_EMPTY_MASK          (0x1 << IRQ_TX_FIFO_EMPTY)
#define IRQ_RX_FIFO_TT                  6         // Receive FIFO threshold reached
#define IRQ_RX_FIFO_TT_MASK             (0x1 << IRQ_RX_FIFO_TT)
#define IRQ_RX_FIFO_UR                  7         // Receive FIFO underrun
#define IRQ_RX_FIFO_UR_MASK             (0x1 << IRQ_RX_FIFO_UR)
#define IRQ_RX_FIFO_OR                  8         // Receive FIFO overrun
#define IRQ_RX_FIFO_OR_MASK             (0x1 << IRQ_RX_FIFO_OR)
#define IRQ_RX_FIFO_FULL                9         // Receive FIFO full
#define IRQ_RX_FIFO_FULL_MASK           (0x1 << IRQ_RX_FIFO_FULL)
#define IRQ_RX_FIFO_EMPTY               10        // Receive FIFO empty
#define IRQ_RX_FIFO_EMPTY_MASK          (0x1 << IRQ_RX_FIFO_EMPTY)
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_FLUSH
//0-------------------------------------------------------------------------------
#define TX_FIFO_FLUSH                   0         // Transmit Data FIFO flush
#define TX_FIFO_FLUSH_MASK              (0x1 << TX_FIFO_FLUSH)
#define RX_FIFO_FLUSH                   1         // Receive Data FIFO flush
#define RX_FIFO_FLUSH_MASK              (0x1 << RX_FIFO_FLUSH)
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_THRESHOLD
//0-------------------------------------------------------------------------------
#define TX_FIFO_THRESHOLD               0         // Transmit FIFO threshold (bits 0–7)
#define TX_FIFO_THRESHOLD_MASK          (0xFF << TX_FIFO_THRESHOLD)
#define RX_FIFO_THRESHOLD               8         // Receive FIFO threshold (bits 8–15)
#define RX_FIFO_THRESHOLD_MASK          (0xFF << RX_FIFO_THRESHOLD)
//0-------------------------------------------------------------------------------
// Defines for subfields of CSR FIFO_STATUS
//0-------------------------------------------------------------------------------
#define TX_FIFO_DEPTH                   0         // bits 0–6
#define TX_FIFO_DEPTH_MASK              (0x7F << TX_FIFO_DEPTH)
#define RX_FIFO_DEPTH                   7         // bits 7–13
#define RX_FIFO_DEPTH_MASK              (0x7F << RX_FIFO_DEPTH)
#define TX_FIFO_AVAIL                   14        // bits 14–20
#define TX_FIFO_AVAIL_MASK              (0x7F << TX_FIFO_AVAIL)
#define RX_FIFO_USED                    21        // bits 21–27
#define RX_FIFO_USED_MASK               (0x7F << RX_FIFO_USED)
#define RX_FIFO_FULL                    28        // bit 28
#define RX_FIFO_FULL_MASK               (0x1 << RX_FIFO_FULL)
#define RX_FIFO_EMPTY                   29        // bit 29
#define RX_FIFO_EMPTY_MASK              (0x1 << RX_FIFO_EMPTY)
#define TX_FIFO_FULL                    30        // bit 30
#define TX_FIFO_FULL_MASK               (0x1 << TX_FIFO_FULL)
#define TX_FIFO_EMPTY                   31        // bit 31
#define TX_FIFO_EMPTY_MASK              (0x1 << TX_FIFO_EMPTY)
#define MCLK 1200000 //12MHZ

//calculated for 1.53 BCLK
void Cfg_Tx_config()
{
    int val=0;
    val |= 1<< TX_ENABLE;

    val |= ( 2 << TX_NUM_CHANNELS );           
    val |= ( 16 << TX_CHANNEL_WIDTH); //by default also 16
    val |= ( 16 << TX_DATA_WIDTH); 
   // val |= ( 0 << TX_PADDING); //No padding needed
    val |= ( 0 <<  TX_PROTO_MODE ); //0-I2S 1-TDM
    val |= ( 1 <<  TX_BIT_ORDER );  //MSB
    val |= ( 0 << TX_JUSTIFICATION); //left justification
    val |= ( 0 << TX_SD_DELAY); //MSB starts immidiately after WS change
    val |=  ( 0 <<TX_CLK_MODE ); //Master
    val |=  ( 0 << TX_CLK_POL); //nedge

    write_reg(TX_CONFIG,val); 
}

void Cfg_Rx_config()
{

    int val=0;
    val |= 1<< RX_ENABLE;

    val |= ( 2 << RX_NUM_CHANNELS );           
    val |= ( 16 << RX_CHANNEL_WIDTH); //by default also 16
    val |= ( 16 << RX_DATA_WIDTH); 
    val |= ( 0 <<  RX_PROTO_MODE ); //0-I2S 1-TDM
    val |= ( 1 <<  RX_BIT_ORDER );  //MSB
    val |= ( 0 << RX_JUSTIFICATION); //left justification
    val |= ( 0 << RX_SD_DELAY); //MSB starts immidiately after WS change
    val |=  ( 0 <<RX_CLK_MODE ); //Master
    val |=  ( 0 << RX_CLK_POL); //nedge

    write_reg(RX_CONFIG,val);
    
}




/*Init I2S controller*/
void init_i2s_controller()
{
    /*Calculate required MCLK and BCLK divider*/
    int m_div = MCLK/req_rate;
    //TX_SCK  = Fs × bits_per_sample × channels
    int BCLK = 48000 * 32 * 2 = 3.072 MHz
    int ratio = MCLK/BCLK;

    /*Configure prescaler register for TX*/
    write_reg( TX_PRESCALAR, m_div|(ratio<<15));
    
    /*Configure prescaler register for RX*/
    write_reg( RX_PRESCALAR, m_div|(ratio<<15));

    /*configure timeout value*/
    write_reg(SOC_TIMEOUT,64);

    /*configure FIFO Threshould*/
    write_reg(FIFO_THRESHOULD,64<<RX_FIFO|64<<TX_FIFO);

    /*Configure TX_CONFIG*/
    Cfg_Tx_config();
    /*Configure RX_CONFIG*/
    cfg_Tx_config();

}

int TX_Data_FIFO(short int *buf,short int num_of_samples)
{
    int val;
    for(int i=0;i<num_of_samples;i++)
    {
        while((val=read_reg(FIFO_STATUS)) & (1<<TX_FIFO_FULL));
        write_reg(TX_FIFO,buf[i]);
    }
    return 0;
}

int RX_Data_FIFO(char *buf,int num)
{
    int val;
     while((val=read_reg(FIFO_STATUS)) & (1<<RX_FIFO_EMPTY));
     char data = read_reg(TX_FIFO);
}

