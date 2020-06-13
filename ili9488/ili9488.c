
#include "sdkconfig.h"
#if defined(CONFIG_DISPLAY_ILI9488)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "../include/display.h"
#include "../common/display_spi_interface.h"

#define ILI9488_CMD_NOP                 (0x00)
#define ILI9488_CMD_COLUMN_ADDR_SET     (0x2A)
#define ILI9488_CMD_PAGE_ADDR_SET       (0x2B)
#define ILI9488_CMD_MEMORY_WRITE        (0x2C)
#define ILI9488_CMD_MEMORY_WRITE_CONT   (0x3C)

/*
 * The LCD needs a bunch of command/argument values to be initialized.
 * They are stored in this struct.
 */
typedef struct {
    uint8_t cmd;
    uint8_t data[8];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_cmd_t;

static const char * TAG = "ili9488";
static bool bInit = false;

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DRAM_ATTR static const lcd_cmd_t ili_init_cmds[]={
#if (CONFIG_DISPLAY_H_RES <= CONFIG_DISPLAY_V_RES) /* Portrait */
    /* Memory access control, MY=0, MX=1, MV=0, ML=0, BGR=1, MH=0 */
    {0x36, {0x48}, 1},
#else   /* Landscape (90deg CW)*/
    /* Memory access control, MY=1, MX=1, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0xE8}, 1},
#endif
    /* Pixel format, 18bits/pixel for RGB 3-wire SPI */
    {0x3A, {0x66}, 1},
    /* Column address set, SC=0, EC=0x140 */
    {0x2A, {0x00, 0x00, 0x01, 0x40}, 4},
    /* Page address set, SP=0, EP=0x01E0 */
    {0x2B, {0x00, 0x00, 0x01, 0xE0}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Normal Display ON */
    {0x13, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
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

    /*
     * Initialize LCD
     */
    while(ili_init_cmds[cmd].databytes != 0xFF) {
        ret = display_interface_write_command(ili_init_cmds[cmd].cmd,
                ili_init_cmds[cmd].data, (ili_init_cmds[cmd].databytes & 0x1F));
        ESP_ERROR_CHECK(ret);
        if(ili_init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }

    bInit = true;
    return ESP_OK;
}

esp_err_t display_write_pixel(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye, rgb_t *pBuf, uint32_t nPixel)
{
    esp_err_t ret = ESP_OK;
    lcd_cmd_t lcd_cmd[3];
    uint32_t i;
    uint32_t nPx;

    if(bInit != true) {
        ESP_LOGE(TAG, "%s:%d: not initialized!\n", __FUNCTION__, __LINE__);
        return ESP_ERR_INVALID_STATE;

    }

    if((xs > xe) || (ys > ye) || ((rgb_t *)0 == pBuf)) {
        ESP_LOGE(TAG, "%s:%d: Invalid Arg!\n", __FUNCTION__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    /* Command: Column Address Set */
    lcd_cmd[0].cmd = ILI9488_CMD_COLUMN_ADDR_SET;
    lcd_cmd[0].data[0] = (uint8_t)((xs >> 8) & 0x00FF);
    lcd_cmd[0].data[1] = (uint8_t)(xs & 0x00FF);
    lcd_cmd[0].data[2] = (uint8_t)((xe >> 8) & 0x00FF);
    lcd_cmd[0].data[3] = (uint8_t)(xe & 0x00FF);
    lcd_cmd[0].databytes = 4;
    /* Command: Page Address Set */
    lcd_cmd[1].cmd = ILI9488_CMD_PAGE_ADDR_SET;
    lcd_cmd[1].data[0] = (uint8_t)((ys >> 8) & 0x00FF);
    lcd_cmd[1].data[1] = (uint8_t)(ys & 0x00FF);
    lcd_cmd[1].data[2] = (uint8_t)((ye >> 8) & 0x00FF);
    lcd_cmd[1].data[3] = (uint8_t)(ye & 0x00FF);
    lcd_cmd[1].databytes = 4;
    /* Command: Write Memory */
    lcd_cmd[2].cmd = ILI9488_CMD_MEMORY_WRITE;
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