
#define THIS_MODULE "mfr_i2c"
#define I2C_CONTROL_REG    0x00
#define I2C_STATUS_REG     0x04
#define I2C_DLEN_REG       0x08
#define I2C_SLAVE_ADDR_REG 0x0C
#define I2C_DATA_FIFO_REG  0x10
#define I2C_CLK_DEV_REG    0x14
#define I2C_DATA_DELAY_REG 0x18
#define I2C_CLK_STRETCH    0x1C

//I2C control Register bits
#define I2C_C_READ  BIT(0) //Read Bit
#define I2C_C_CLEAR BIT(4) //BIT 4 & 5.
#define I2C_C_ST    BIT(7) //Start Xfer.
#define I2C_C_INTD  BIT(8) //Int on Done.
#define I2C_C_INTT  BIT(9) //Int on Xfer.
#define I2C_C_INTR  BIT(10) //Int on RX.
#define I2C_C_EN    BIT(15)  //Enable I2C.

//I2C Status Register
#define I2C_S_TA	BIT(0) //Xfer Active.
#define I2C_S_DONE	BIT(1) //Xfer Done.
#define I2C_S_TXW	BIT(2) //FIFO needs writing.
#define I2C_S_RXR	BIT(3) //FIFO needs Reading.
#define I2C_S_TXD	BIT(4) //Accept Data.
#define I2C_S_RXD	BIT(5) //Contains Data.
#define I2C_S_TXE	BIT(6) //Empty.
#define I2C_S_RXF	BIT(7) //Full.
#define I2C_S_ERR	BIT(8) //ACK err.
#define I2C_S_CLKT	BIT(9) //Clock Stretch Timeout.

//I2C DEL(Delay) Register
#define I2C_FEDL_SHIFT	16 //Falling Edge Delay.
#define I2C_REDL_SHIFT	0  //Rising Edge Delay.

//I2C CDIV Register
/* Unreferenced */
#define I2C_CDIV_MIN	0x0002
#define I2C_CDIV_MAX	0xFFFE
