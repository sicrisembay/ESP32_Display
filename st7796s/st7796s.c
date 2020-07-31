
#include "sdkconfig.h"
#if defined(CONFIG_DISPLAY_ST7796S)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "../include/display.h"
#include "../common/display_spi_interface.h"

#define ST7796S_CMD_NOP                 (0x00)
#define ST7796S_CMD_COLUMN_ADDR_SET     (0x2A)
#define ST7796S_CMD_PAGE_ADDR_SET       (0x2B)
#define ST7796S_CMD_MEMORY_WRITE        (0x2C)
#define ST7796S_CMD_MEMORY_WRITE_CONT   (0x3C)

/*
 * The LCD needs a bunch of command/argument values to be initialized.
 * They are stored in this struct.
 */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_cmd_t;

static const char * TAG = "st7796s";
static bool bInit = false;
static DMA_ATTR lcd_cmd_t lcd_cmd[3];

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DRAM_ATTR static const lcd_cmd_t st_init_cmds[]={
    /* SW Reset */
    {0x01, {0}, 0x80},
    /* Command Set Control: Enable Command2 */
    {0xF0, {0xC3}, 1},
    {0xF0, {0x96}, 1},
#if (CONFIG_DISPLAY_H_RES <= CONFIG_DISPLAY_V_RES) /* Portrait */
    /* Memory access control, MY=0, MX=1, MV=0, ML=0, BGR=1, MH=0 */
    {0x36, {0x48}, 1},
#else   /* Landscape (90deg CW)*/
    /* Memory access control, MY=1, MX=1, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0xE8}, 1},
#endif
    /* Interface Pixel Format: 16bits/pixel for RGB and MCU interface */
    {0x3A, {0x55}, 1},
    /* Display Function Control */
    {0xB6, {0x80, 0x02, 0x3B}, 3},
    /* Display Output Ctrl Adjust */
    {0xE8, {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33}, 8},
	/* Power control2 */
    {0xC1, {0x06}, 1},
    /* Power control 3 */
    {0xC2, {0xA7}, 1},
    /* VCOM Control */
    {0xC5, {0x18}, 0x81},  // with delay
	/* ST7796 Gamma Sequence */
    {0xE0, {0xF0, 0x09, 0x0b, 0x06, 0x04, 0x15, 0x2F, 0x54, 
	        0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B}, 14}, 
	{0xE1, {0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43,
	        0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B}, 0x80 + 14}, // with delay
	/* Command Set control: Disable extension command 2 */
    {0xF0, {0x3C}, 1},
    {0xF0, {0x69}, 1},
    /* Sleep Out */
    {0x11, {0}, 0x80},
    /* Idle Mode OFF */
    {0x38, {0}, 0x80},
    /* Normal Display ON */
    {0x13, {0}, 0x80},
    /* Display On */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff}
};

esp_err_t display_init(void)
{
    esp_err_t ret = ESP_OK;
    gpio_config_t io_config;
    int cmd = 0;

    /*
     * Initialize Backlight pin
     */
#if (defined(CONFIG_DISPLAY_BACKLIGHT_IO_PIN) && (CONFIG_DISPLAY_BACKLIGHT_IO_PIN != -1))
    io_config.intr_type = GPIO_PIN_INTR_DISABLE;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pin_bit_mask = (1ULL << CONFIG_DISPLAY_BACKLIGHT_IO_PIN);
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 0;
    ret = gpio_config(&io_config);
    ESP_ERROR_CHECK(ret);
    gpio_set_level(CONFIG_DISPLAY_BACKLIGHT_IO_PIN, 0);
#endif

    /*
     * Initialize Reset
     */
#if (defined(CONFIG_DISPLAY_RESET_IO_PIN) && (CONFIG_DISPLAY_RESET_IO_PIN != -1))
    io_config.intr_type = GPIO_PIN_INTR_DISABLE;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pin_bit_mask = (1ULL << CONFIG_DISPLAY_RESET_IO_PIN);
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 0;
    ret = gpio_config(&io_config);
    ESP_ERROR_CHECK(ret);
    gpio_set_level(CONFIG_DISPLAY_RESET_IO_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS); /* 100ms */
    /* Perform LCD Reset */
    gpio_set_level(CONFIG_DISPLAY_RESET_IO_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS); /* 100ms */
    gpio_set_level(CONFIG_DISPLAY_RESET_IO_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS); /* 100ms */
#endif

    ret = display_interface_init();
    ESP_ERROR_CHECK(ret);

    /*
     * Initialize LCD
     */
    while (st_init_cmds[cmd].databytes!=0xff) {
        ret = display_interface_write_command(st_init_cmds[cmd].cmd,
                    st_init_cmds[cmd].data, (st_init_cmds[cmd].databytes & 0x1F));
        ESP_ERROR_CHECK(ret);
        if (st_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    bInit = true;
    return ESP_OK;
}

esp_err_t display_write_pixel(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye, uint8_t *pBuf, uint32_t nPixel)
{
    esp_err_t ret = ESP_OK;
    uint32_t i;
    uint32_t nPx;

    if(bInit != true) {
        ESP_LOGE(TAG, "%s:%d: not initialized!\n", __FUNCTION__, __LINE__);
        return ESP_ERR_INVALID_STATE;

    }

    if((xs > xe) || (ys > ye) || ((uint8_t *)0 == pBuf)) {
        ESP_LOGE(TAG, "%s:%d: Invalid Arg!\n", __FUNCTION__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    /* Command: Column Address Set */
    lcd_cmd[0].cmd = ST7796S_CMD_COLUMN_ADDR_SET;
    lcd_cmd[0].data[0] = (uint8_t)((xs >> 8) & 0x00FF);
    lcd_cmd[0].data[1] = (uint8_t)(xs & 0x00FF);
    lcd_cmd[0].data[2] = (uint8_t)((xe >> 8) & 0x00FF);
    lcd_cmd[0].data[3] = (uint8_t)(xe & 0x00FF);
    lcd_cmd[0].databytes = 4;
    /* Command: Page Address Set */
    lcd_cmd[1].cmd = ST7796S_CMD_PAGE_ADDR_SET;
    lcd_cmd[1].data[0] = (uint8_t)((ys >> 8) & 0x00FF);
    lcd_cmd[1].data[1] = (uint8_t)(ys & 0x00FF);
    lcd_cmd[1].data[2] = (uint8_t)((ye >> 8) & 0x00FF);
    lcd_cmd[1].data[3] = (uint8_t)(ye & 0x00FF);
    lcd_cmd[1].databytes = 4;
    /* Command: Write Memory */
    lcd_cmd[2].cmd = ST7796S_CMD_MEMORY_WRITE;
    lcd_cmd[2].data[0] = 0;
    lcd_cmd[2].databytes = 0;

    for(i = 0; i < 3; i++) {
        ret = display_interface_write_command(lcd_cmd[i].cmd,
                    lcd_cmd[i].data, (lcd_cmd[i].databytes & 0x1F));
        if(lcd_cmd[i].databytes & 0x80) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        if(ESP_OK != ret) {
            ESP_LOGE(TAG, "%s:%d: Err%d\n", __FUNCTION__, __LINE__, ret);
            return ret;
        }
    }

    /* Calculate number of pixels from column and raw info */
    nPx = (xe - xs + 1) * (ye - ys + 1);
    if(nPx > nPixel) {
        nPx = nPixel;
    }

    ret = display_interface_write_px(pBuf, nPixel);
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "%s:%d: Err%d\n", __FUNCTION__, __LINE__, ret);
        return(ret);
    }
    return ESP_OK;
}

esp_err_t display_backlight_set(bool state)
{
    esp_err_t ret = ESP_OK;
    if(bInit != true) {
        ESP_LOGE(TAG, "%s:%d: not initialized!\n", __FUNCTION__, __LINE__);
        return ESP_ERR_INVALID_STATE;
    }
    if(state == true) {
        ret = gpio_set_level(CONFIG_DISPLAY_BACKLIGHT_IO_PIN, 1);
    } else {
        ret = gpio_set_level(CONFIG_DISPLAY_BACKLIGHT_IO_PIN, 0);
    }

    return ret; 
}

#endif /* CONFIG_ESP32_DISPLAY */