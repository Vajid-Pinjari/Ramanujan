#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>

/*  File Operations and character device    */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include "../inc/sh1106.h"

/* header for own logger */
#include "log_header.h" 


#define AVAILABLE_RPI_I2C_BUS           1U
#define SSH1306_OLED_ADR                0x3C
#define CLIENT_NAME                     "i2c_client_sh1106"

#define OK                              0U

#define CLASS_NAME                      "OLED"

/*  Major and minor number for exposing the driver to the userspace */
static int major_number;

/*  Will be used to register the fops to the cdev, will be further added to the dev */
static struct cdev *oled_cdev = NULL;

/*  Initialize the display buffer depending upon the display ht and wdt */
static unsigned char display_buffer[BUFFER_SIZE];

static unsigned char bird[BUFFER_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x70, 0xf0, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 
        0xf8, 0xf8, 0xf8, 0xf0, 0xf0, 0xf0, 0xe0, 0xe0, 0xc0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x60, 0x70, 0x70, 0xf8, 0xf8, 0xfc, 0xfc, 0xfd, 0xff, 0xff, 0xff, 0xff, 
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0xe0, 
        0x80, 0x80, 0xc0, 0xc0, 0xe0, 0xe0, 0xf0, 0xf0, 0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xf8, 0xf8, 0xf0, 
        0xf0, 0xe0, 0x80, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
        0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x0f, 0x07, 
        0x07, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x01, 0x01, 0x03, 0x03, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
        0xff, 0xff, 0xff, 0x7f, 0x7f, 0x3f, 0x1f, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xdf, 0x9f, 0x8f, 0x0f, 0x0f, 0x07, 0x07, 0x03, 
        0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x03, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
        0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0f, 0x0f, 0x1f, 
        0x1f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
// display_buffer[0] = (unsigned char)OLED_CNT_BYTE_DATA_STREAM;

/*  display initialization commands */
unsigned char display_off_cmd[] = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_DISPLAY_OFF};
unsigned char clk_div_cmd[]         = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_DISPLAY_CLK_DIV, 0x80};
unsigned char multiplex_ratio[]     = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_MULTIPLEX_RATIO, 0x3F};
unsigned char display_offset[]      = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_DISPLAY_OFFSET, 0x00};
unsigned char display_start_line[]  = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_DISPLAY_START_L};
unsigned char unknown_cmd_1[]       = {OLED_CNT_BYTE_CMD_STREAM, 0x8D, 0x14};
unsigned char unknown_cmd_2[]       = {OLED_CNT_BYTE_CMD_STREAM, 0x20, 0x00};
unsigned char segment_remap[]       = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_SEG_REMAP_H};
unsigned char scan_dir[]            = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_COM_OP_SCAN_V};
unsigned char com_hw_pad[]          = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_COM_PAD_HW, 0x12};
unsigned char contrast[]            = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_CONTRAST, 0xCF};
unsigned char precharge[]           = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_PRECHARGE, 0xF1};
unsigned char vcom[]                = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_VCOM_LEVEL, 0x40};
unsigned char unknown_cmd_3[]       = {OLED_CNT_BYTE_CMD_STREAM, 0x2E};
unsigned char display_style[]       = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_ENT_DISPLAY_RAM};
unsigned char display_style1[]      = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_DISPLAY_NORMAL};
unsigned char setpage[]             = {OLED_CNT_BYTE_CMD_STREAM, 0x10, 0x2, OLED_SET_PAGE_ADR};
unsigned char display_on_cmd[]      = {OLED_CNT_BYTE_CMD_STREAM, OLED_SET_DISPLAY_ON};

/*  declare and define a adapter structure to which the device will be connected on */
static struct i2c_adapter *oled_i2c_adap = NULL;
static struct i2c_client *oled_i2c_client = NULL;

/*  Dynamic node creation   
static struct class *oled_class;
static struct device *oled_device;
*/

/* IOCTL Commands defined
#define CLEAR_SH1106    _IO('a', 1)  // Command to clear the SH1106 display
#define DRAW_PIXEL      _IO('a', 2)   // Command to draw a pixel on the SH1106 display
*/

/**
 *  Write message to the i2c_bus, takes a pointer to the char data type.
 */
static int sh1106_write(const unsigned char *buf, int size)
{
    msleep(50);
    int xRet = OK;

    if(!oled_i2c_client){
        pr_err("%s: oled_i2c_client!\n", CLIENT_NAME); 
        log_error("%s: oled_i2c_client!", CLIENT_NAME); 

        return -ENODATA;
    }
    xRet = i2c_master_send(oled_i2c_client, buf, size);
    if(xRet < 0){
        pr_err("%s: Error with i2c_master_send: %d", CLIENT_NAME, xRet);
        return xRet;
    }
    return 0;
}
/**
 *  Send the display initialization commands to the oled display
 * 
 */
static int SSH1106_display_init(void)
{
    int xRet = OK;

    if ((xRet = sh1106_write(display_off_cmd, sizeof(display_off_cmd))))        goto err_dump;
    if ((xRet = sh1106_write(clk_div_cmd, sizeof(clk_div_cmd))))                goto err_dump;
    if ((xRet = sh1106_write(multiplex_ratio, sizeof(multiplex_ratio))))        goto err_dump;
    if ((xRet = sh1106_write(display_offset, sizeof(display_off_cmd))))         goto err_dump;
    if ((xRet = sh1106_write(display_start_line, sizeof(display_start_line))))  goto err_dump;
    if ((xRet = sh1106_write(unknown_cmd_1, sizeof(unknown_cmd_1))))            goto err_dump;
    if ((xRet = sh1106_write(unknown_cmd_2, sizeof(unknown_cmd_2))))            goto err_dump;
    if ((xRet = sh1106_write(segment_remap, sizeof(segment_remap))))            goto err_dump;
    if ((xRet = sh1106_write(scan_dir, sizeof(scan_dir))))                      goto err_dump;
    if ((xRet = sh1106_write(com_hw_pad, sizeof(com_hw_pad))))                  goto err_dump;
    if ((xRet = sh1106_write(contrast, sizeof(contrast))))                      goto err_dump;
    if ((xRet = sh1106_write(precharge, sizeof(precharge))))                    goto err_dump;
    if ((xRet = sh1106_write(vcom, sizeof(vcom))))                              goto err_dump;
    if ((xRet = sh1106_write(unknown_cmd_3, sizeof(unknown_cmd_3))))            goto err_dump;
    if ((xRet = sh1106_write(display_style, sizeof(display_style))))            goto err_dump;
    if ((xRet = sh1106_write(display_style1, sizeof(display_style1))))          goto err_dump;
    if ((xRet = sh1106_write(setpage, sizeof(setpage))))                        goto err_dump;
    return 0;

err_dump:
    return xRet;
}

static int sh1106_clear_oled(void)
{
    msleep(10);
    int xRet = OK;

    /*  Create the buffer to tx the 0s  */
    uint8_t buff_fill_zeros[25] = {0};
    buff_fill_zeros[0] = OLED_CNT_BYTE_DATA_STREAM;

    uint8_t total_seg_batch = 6;
    uint8_t count = 0;
    for (uint8_t page_start = 0; page_start < 8; page_start++)
    {
        setpage[3] = OLED_SET_PAGE_ADR + count++;

        xRet = sh1106_write(display_start_line, sizeof(display_start_line));
        if(xRet < 0){
            pr_err("%s: error in clearing the display, error code = %d!\n", CLIENT_NAME, xRet);               
            log_error("%s: error in clearing the display, error code = %d!", CLIENT_NAME, xRet);

            goto err_dump;
        }

        xRet = sh1106_write(setpage, sizeof(setpage));
        if(xRet < 0){
            pr_err("%s: error in clearing the display, error code = %d!\n", CLIENT_NAME, xRet);
            log_error("%s: error in clearing the display, error code = %d!", CLIENT_NAME, xRet);

            goto err_dump;
        }

        for (uint8_t seg_batch = 1; seg_batch <= total_seg_batch; seg_batch++){
            uint8_t seg_count;
            if(seg_batch == 6){
                seg_count = 9;
            }
            else{
                seg_count = 25;
            }

            xRet = sh1106_write(buff_fill_zeros, seg_count);
            if (xRet < 0){
                pr_err("%s: error in clearing the display, error code = %d!\n", CLIENT_NAME, xRet);
                log_error("%s: error in clearing the display, error code = %d!", CLIENT_NAME, xRet);

                goto err_dump;
            }

            //msleep(5);
        }
        //msleep(5);
    }

    return 0;

    err_dump:
        return xRet;
}

static int sh1106_oled_draw_pixed(int x_pos, int y_pos, bool status){
    int xRet = OK;

    /*  Check for the appropriate x and y cordinates    */
    if(x_pos < 0 || x_pos > SH1106_WIDTH){
        pr_err("%s: wrong x coordinates!\n", CLIENT_NAME);
        log_error("%s: wrong x coordinates!\n", CLIENT_NAME);

        goto err_dump;
    }
    if(y_pos < 0 || y_pos > SH1106_HEIGHT){
        pr_err("%s: wrong y coordinates!\n", CLIENT_NAME);
        log_error("%s: wrong y coordinates!", CLIENT_NAME);

        goto err_dump;
    }

    int page = y_pos / SH1106_PAGES;
    int y_bit_pos = y_pos % SH1106_PAGES;

    /*  index of the 1024 byte buffer depending upon x and y coordinate */
    int byte_index = x_pos + (page * SH1106_WIDTH);

    pr_info("Drawing pixel at byte_index=%d, page=%d, y_bit_pos=%d\n", byte_index, page, y_bit_pos);
    log_info("Drawing pixel at byte_index=%d, page=%d, y_bit_pos=%d", byte_index, page, y_bit_pos);

    /*  Depending upon the status turn on or off the pixel  */
    if(status){
        /*  Turn on the display */
        display_buffer[byte_index] |= (1 << y_bit_pos);
    }
    else{
        /*  Turn off the display    */
        display_buffer[byte_index] &= ~(1 << y_bit_pos);
    }

    xRet = update_display(display_buffer);

    return 0;

    err_dump:
        return -EFAULT;
}

int update_display(const unsigned char *free_buffer){

#define OLED_UPDATE_CHUNK_SIZE          32 // Parameterized chunk size
#define OLED_UPDATE_BUFFER_SIZE         (OLED_UPDATE_CHUNK_SIZE + 1)

    int xRet = OK;
    uint8_t page_lookup = 0, page_addr_count = 0;

    uint8_t update_buffer[OLED_UPDATE_BUFFER_SIZE] = {0};
    update_buffer[0] = OLED_CNT_BYTE_DATA_STREAM;

    /* Iterate through the display buffer in chunks */
    for (uint16_t count = 0; count < 1024; count += OLED_UPDATE_CHUNK_SIZE){
        
        if ((page_lookup == 0) || ((page_lookup % 4) == 0)){
            setpage[3] = OLED_SET_PAGE_ADR + page_addr_count++;

            xRet = sh1106_write(display_start_line, sizeof(display_start_line));
            if (xRet < 0){
                pr_err("%s: error in updating the display, error code = %d!\n", CLIENT_NAME, xRet);
                log_error("%s: error in updating the display, error code = %d!", CLIENT_NAME, xRet);

                goto err_dump;
            }

            xRet = sh1106_write(setpage, sizeof(setpage));
            if (xRet < 0){
                pr_err("%s: error in updating the display, error code = %d!\n", CLIENT_NAME, xRet);
                log_error("%s: error in updating the display, error code = %d!", CLIENT_NAME, xRet);

                goto err_dump;
            }
        }
        
        /* Copy the next chunk of data into the update buffer */
        memcpy(&update_buffer[1], &(free_buffer[count]), OLED_UPDATE_CHUNK_SIZE);

        /* Send the chunk over I2C */
        xRet = sh1106_write(update_buffer, OLED_UPDATE_BUFFER_SIZE);

        if (xRet < 0){
            pr_err("%s: Error updating display at chunk %d!\n", CLIENT_NAME, count / OLED_UPDATE_CHUNK_SIZE);
            return xRet;
        }
/* need to change the log info  */
        pr_info("Chunk sent: %*ph\n", OLED_UPDATE_CHUNK_SIZE, &update_buffer[1]);
        log_info("Chunk sent: %ph", OLED_UPDATE_CHUNK_SIZE, &update_buffer[1]);

        page_lookup++;

    }

    return OK;

    err_dump:
        return xRet;
}

/********************************************************************************************
 *  Exposing the fops to userspace so that the user can control the display using the file operations
 * 
static int oled_open_from_up(struct inode *pInode, struct file *pFile){
    pr_info("%s: opened!\n", CLIENT_NAME);
    return 0;
}

static int oled_release_from_up(struct inode *pInode, struct file *pFile){
    pr_info("%s: closed!\n", CLIENT_NAME);
    return 0;
}

static long oled_ioctl_up(struct file *pFile, unsigned int cmd, unsigned long arg){
    switch(cmd){
        case CLEAR_SH1106:{
            pr_info("%s: Inside clear_sh1006 ioctl!\n", CLIENT_NAME);
            break;
        }
        case DRAW_PIXEL:{
            pr_info("%s: Inside draw_pixel ioctl!\n", CLIENT_NAME);
            break;
        }
    }

    return 0;
}
*/

/*  Declare and initialize the fops with custom functions    
static struct file_operations fops_on_oled = {
    .owner = THIS_MODULE,
    .open = oled_open_from_up,
    .release = oled_release_from_up,
    .unlocked_ioctl = oled_ioctl_up,
};
*/



/*********************************************************************************************/
/**
*  Will be invoked if the client device is found on the desired i2c bus
*  Will only be called once
*/
static int oled_probe(struct i2c_client * client){
    int xRet = OK;

    /*  Initialize the display  */
    xRet = SSH1106_display_init();
    if (xRet < 0){
        pr_err("%s: error in initializing the sh1106 display!\n", CLIENT_NAME);
        log_error("%s: error in initializing the sh1106 display!", CLIENT_NAME);

        goto dump_error;
    }

    /*  Clear the display   */
    sh1106_clear_oled();
    pr_info("%s: Cleared the display GDDRAM!\n", CLIENT_NAME);
    log_info("%s: Cleared the display GDDRAM!", CLIENT_NAME);

    /*  Turn a pixel on 
    xRet = sh1106_oled_draw_pixed(64, 32, true);
    if(xRet < 0){
        pr_err("%s: unable to turn on the pixel!\n", CLIENT_NAME);
        goto dump_error;
    }   
    pr_info("%s: turned on/off the pixel at (%d, %d)!\n", CLIENT_NAME, 64, 32);
    */

    /* Draw the bird    */
    xRet = update_display(bird);
    if(xRet < 0){
        pr_err("%s: unable to write to the buffer!\n", CLIENT_NAME);
        log_error("%s: unable to write to the buffer!", CLIENT_NAME);

        goto dump_error;
        
    }
    pr_info("%s: successfully written to the buffer!\n", CLIENT_NAME);
    log_info("%s: successfully written to the buffer!", CLIENT_NAME);



    /*  Turn on the display */
    sh1106_write(display_on_cmd, sizeof(display_on_cmd));
    pr_info("%s: Turned on the display!\n", CLIENT_NAME);
    log_info("%s: Turned on the display!", CLIENT_NAME);

    pr_info("%s: probed!\n", CLIENT_NAME);
    log_info("%s: probed!", CLIENT_NAME);

    return 0;

    dump_error:
        return xRet;
}

static void oled_remove(struct i2c_client *client){
/*  Apply a mechanism to control something on the oled display before getting removed   */
    sh1106_write(display_off_cmd, sizeof(display_off_cmd));

    pr_info("%s: removed!\n", CLIENT_NAME);
    log_info("%s: removed!", CLIENT_NAME);
}
/*********************************************************************************************/

/*  Structure containing client id, used for registration   */
static const struct i2c_device_id oled_id[] = {
    { .name = CLIENT_NAME},
    { }
};
MODULE_DEVICE_TABLE(i2c, oled_id);

/*  Instantiate the <struct i2c_driver> for the device  */
static struct i2c_driver oled_driver = {
    .driver = {
        .name = CLIENT_NAME,
        .owner = THIS_MODULE,
    },
    .probe = oled_probe,
    .remove = oled_remove,
    .id_table = oled_id,
};

/*  Specify board info to get the i2c_client    */
static struct i2c_board_info oled_i2c_board_info = {
        I2C_BOARD_INFO(CLIENT_NAME, SSH1306_OLED_ADR),
        /*  SCL Frequency for the slave/client */
        .platform_data = (void *)400000,
};

/*********************************************************************************************/
/*  OLED Module init function   */
static int __init oled_driver_init(void){
    int xRet = OK;

    /*  Get the i2c_adapter structure for the specified i2c bus */
    oled_i2c_adap = i2c_get_adapter(AVAILABLE_RPI_I2C_BUS);
    if(!oled_i2c_adap){
        pr_err("%s: Failed to get the i2c_adapter for the i2c%d bus!\n", CLIENT_NAME, AVAILABLE_RPI_I2C_BUS);
        /* own logger implementation */
        log_error("%s: Failed to get the i2c_adapter for the i2c%d bus!", CLIENT_NAME, AVAILABLE_RPI_I2C_BUS);
        goto err_dump;
    }

    oled_i2c_client = i2c_new_client_device(oled_i2c_adap, &oled_i2c_board_info);
    if(!oled_i2c_client){
        pr_err("%s: Failed to register the i2c_client!\n", CLIENT_NAME);

        log_error("%s: Failed to register the i2c_client!", CLIENT_NAME);
        goto err_dump;
    }

    i2c_put_adapter(oled_i2c_adap);

    i2c_add_driver(&oled_driver);


    /*  Get the pointer to the cdev structure   */
    oled_cdev = cdev_alloc();

    /*  Initialize the cdev with fops   */
    cdev_init(oled_cdev, &fops_on_oled);

    


    /*  Fops related registration
        Registeration of the driver as a character device
    
    major_number = register_chrdev(0, CLIENT_NAME, &fops_on_oled);
    if(major_number < 0){
        pr_err("%s: Failed to register, error_code = %d", CLIENT_NAME, major_number);
        return major_number;
    }
    */


    /*  Create a class for the device 
    oled_class = class_create(CLASS_NAME);
    if(IS_ERR(oled_class)){
        pr_err("%s: Failed to create the class!\n", CLIENT_NAME);
        unregister_chrdev(major_number, CLIENT_NAME);
        return PTR_ERR(oled_class);
    }
    */

    /*  Create a device
    oled_device = device_create(oled_class, NULL, MKDEV(major_number, 0), NULL, CLIENT_NAME);
    if(IS_ERR(oled_device)){
        pr_err("%s: Failed to create the device!\n", CLIENT_NAME);
        class_destroy(oled_class);
        unregister_chrdev(major_number, CLIENT_NAME);
        return PTR_ERR(oled_device);
    }
    */
    
    pr_info("%s: Driver initialized and loaded!\n", CLIENT_NAME);
    return 0;

    err_dump:
        return xRet;
}

/*  OLED module Exit Function    */
static void __exit oled_driver_exit(void){
    i2c_unregister_device(oled_i2c_client);
    i2c_del_driver(&oled_driver);

    /* Fops related unregistration
    device_destroy(oled_class, MKDEV(major_number, 0));
    class_destroy(oled_class);
    unregister_chrdev(major_number, CLIENT_NAME);
    */

    pr_info("Driver Removed!\n");
    log_info("Driver Removed!");
}
/*********************************************************************************************/

module_init(oled_driver_init);
module_exit(oled_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Team6");
MODULE_DESCRIPTION("Basic Implmentation of I2C Linux Device Driver");
MODULE_VERSION("1.0");
