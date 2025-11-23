#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <inttypes.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"

// #include "lvgl.h"
// #include "lv_port_disp.h"
// #include "lv_port_indev.h"

#define H_PIXELS 240

#define LCD_HOST SPI2_HOST

// #define PIN_NUM_ /MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
// #define PIN_NUM_CS 5

#define PIN_NUM_DC 21
#define PIN_NUM_RST 22

spi_device_handle_t spi;

void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 8;             // Command is 8 bits
    t.tx_buffer = &cmd;       // The data is the cmd itself
    t.user = (void *)0;       // D/C needs to be set to 0
    if (keep_cs_active)
    {
        t.flags = SPI_TRANS_CS_KEEP_ACTIVE; // Keep CS active after data transfer
    }
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

uint8_t f = 0;

void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0)
    {
        return; // no need to send anything
    }
    memset(&t, 0, sizeof(t));                   // Zero out the transaction
    t.length = len * 8;                         // Len is in bytes, transaction length is in bits.
    if(f)
        printf("sizeof(t.length):%d, t.length:%zu, len:%d\n", sizeof(t.length), t.length, len);
    t.tx_buffer = data;                         // Data
    t.user = (void *)1;                         // D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

void clearAll()
{
    int i,j;
    uint8_t d = 0;

    lcd_cmd(spi, 0x04, 0);      //colum
    lcd_data(spi, &d, 1);
    lcd_cmd(spi, 0x60, 0);      //page, LSB
    lcd_cmd(spi, 0x70, 0);      //page, MSB

    for(i=0;i<=239;i++) {	    //page
        for(j=0;j<=11;j++) {	//coulumn
            lcd_cmd(spi, 0x01, 0);
			lcd_data(spi, &d, 1); 
        }
    }
}

/* user custom page*/
void customPageWindowProgramming(uint8_t start_page, uint8_t end_page, uint8_t start_col, uint8_t end_col, uint8_t *data)
{
    lcd_cmd(spi, 0x04, 0);      //colum
    lcd_data(spi, &start_col, 1);

    lcd_cmd(spi, (0x60 | (0x0f & start_page)), 0);          //page, LSB
    lcd_cmd(spi, (0x70 | ((start_page & 0x30) >> 2)), 0);   //page, MSB

    lcd_cmd(spi, 0x01, 0);
    lcd_data(spi, data, (end_page - start_page + 1) * (end_col - start_col + 1)); 
}

void windowsPrograme(uint8_t start_page, uint8_t end_page, uint8_t start_col, uint8_t end_col, uint8_t *data)
{
    printf("func:%s, start_page:%d, end_page:%d, start_col:%d, end_col:%d\n", __FUNCTION__, start_page, end_page, start_col, end_col);

    uint8_t d = 0;

    lcd_cmd(spi, 0xf8, 0);          //TODO:Disable Window Program, This command must have!!!!!!

    //Set Window Programming Column
    lcd_cmd(spi, 0xf4, 0);
    lcd_data(spi, &start_col, 1);   //startx  
    lcd_cmd(spi, 0xf6, 0);
    lcd_data(spi, &end_col, 1);     //endx

    //Set Window Programming Page
    lcd_cmd(spi, 0xf5, 0);          //start page
    d = start_page & 0x3f;          //D6D7 must be 0
    lcd_data(spi, &d, 1);   

    lcd_cmd(spi, 0xf7, 0);          //end page
    d = end_page & 0x3f;            //D6D7 must be 0
    lcd_data(spi, &d, 1);    

    lcd_cmd(spi, 0xf9, 0);          //Enable Window Program

    uint32_t t = 0;
    t = (end_page - start_page + 1) * (end_col - start_col + 1);
    printf("t = %lu\n", t);    
    lcd_cmd(spi, 0x01, 0);
    f = 1;
    for(uint8_t i = 0; i <= end_col - start_col + 1; i++){
        printf("data[%d]:%x\n", i, data[i]);
    }
    lcd_data(spi, data, t);
    f = 0;

    // lcd_cmd(spi, 0xf8, 0);          //TODO:Disable Window Program, This command must have!!!!!!
}

uint8_t lcdDisLib[][16] = 
{
    {0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x0F, 0x10, 0x20, 0x20, 0x10, 0x0F, 0x00}, /*"0",0*/
    {0x00, 0x00, 0x10, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00}, /*"1",1*/
    {0x00, 0x70, 0x08, 0x08, 0x08, 0x08, 0xF0, 0x00, 0x00, 0x30, 0x28, 0x24, 0x22, 0x21, 0x30, 0x00}, /*"2",2*/
    {0x00, 0x30, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x18, 0x20, 0x21, 0x21, 0x22, 0x1C, 0x00}, /*"3",3*/
    {0x00, 0x00, 0x80, 0x40, 0x30, 0xF8, 0x00, 0x00, 0x00, 0x06, 0x05, 0x24, 0x24, 0x3F, 0x24, 0x24}, /*"4",4*/
    {0x00, 0xF8, 0x88, 0x88, 0x88, 0x08, 0x08, 0x00, 0x00, 0x19, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00}, /*"5",5*/
    {0x00, 0xE0, 0x10, 0x88, 0x88, 0x90, 0x00, 0x00, 0x00, 0x0F, 0x11, 0x20, 0x20, 0x20, 0x1F, 0x00}, /*"6",6*/
    {0x00, 0x18, 0x08, 0x08, 0x88, 0x68, 0x18, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x01, 0x00, 0x00, 0x00}, /*"7",7*/
    {0x00, 0x70, 0x88, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x1C, 0x22, 0x21, 0x21, 0x22, 0x1C, 0x00}, /*"8",8*/
    {0x00, 0xF0, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x01, 0x12, 0x22, 0x22, 0x11, 0x0F, 0x00}, /*"9",9*/
};
void testDisp()
{
            static uint32_t cnt = 0;
        static uint8_t s = 0;
        static uint8_t idx = 0;
        cnt++;
        if(cnt >= 200){
            cnt = 0;

            if(!s){
                s = 1;
            }
            else{
                s = 0;
            }
            printf("idx:%d\n", idx);
            windowsPrograme(0, 1, 100, 107, &lcdDisLib[idx][0]);
            windowsPrograme(2, 3, 100, 107, &lcdDisLib[idx][0]);
            windowsPrograme(4, 5, 100, 107, &lcdDisLib[idx][0]);
            windowsPrograme(6, 7, 100, 107, &lcdDisLib[idx][0]);
            windowsPrograme(8, 9, 100, 107, &lcdDisLib[idx][0]);
            windowsPrograme(10, 11, 100, 107, &lcdDisLib[idx][0]);
            idx++;
            if(idx > 9)
                idx = 0;
        }
}

void lcd_spi_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = H_PIXELS * 12};   //240 columns in total, with 8 bytes per column per page
    spi_device_interface_config_t devcfg = {

#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz = 26 * 1000 * 1000, // Clock out at 26 MHz
#else
        .clock_speed_hz = 10 * 1000 * 1000, // Clock out at 10 MHz
#endif
        .mode = 0,                          // SPI mode 0
        .spics_io_num = -1,                 // CS pin
        .queue_size = 10,                         // We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

// Initialize the display
void lcd_init()
{
    lcd_spi_init();

    // Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = true;
    gpio_config(&io_conf);

    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    uint8_t d = 0;
    lcd_cmd(spi, 0xe1, 0);//system reset
    d = 0xe2;
    lcd_data(spi, &d, 1);

    vTaskDelay(2 / portTICK_PERIOD_MS);

    lcd_cmd(spi, 0x024, 0);  /*	 set temp comp*/
    lcd_cmd(spi, 0xC2, 0);   //set lcd mapping control
    lcd_cmd(spi, 0xA2, 0);   //set line rate  20klps

    lcd_cmd(spi, 0xEb, 0);   //set bias=1/12

    lcd_cmd(spi, 0x81, 0);   //Set VBIAS Potentiometer
    d = 120;
    lcd_data(spi, &d, 1);    //pm=106 Set VLCD=15V

    lcd_cmd(spi, 0x95, 0);   // PT0   1B P P

    lcd_cmd(spi, 0xf1, 0);   //set com end
    d = 159;
    lcd_data(spi, &d, 1);    //set com end   240*160

    lcd_cmd(spi, 0x089, 0);  /*	 set auto increment, low bits are AC2 AC1 AC0 */

    clearAll();             //clear all pixel

    lcd_cmd(spi, 0xc9, 0);
    d = 0xad;
    lcd_data(spi, &d, 1);   //  display 
}

