#include "sdkconfig.h"
#include "esp_log.h"
#include "../../../include/display.h"

static const char *TAG = "test_main";

#define TEST_BUF_SIZE  (CONFIG_DISPLAY_H_RES * CONFIG_DISPLAY_PARALLEL_LINE_COUNT)
static rgb_t testColorBuf[TEST_BUF_SIZE];

static void setBufColor(rgb_t color)
{
    int i = 0;
    for(i = 0; i < TEST_BUF_SIZE; i++) {
        testColorBuf[i].blue = color.blue;
        testColorBuf[i].green = color.green;
        testColorBuf[i].red = color.red;
    }
}

void app_main()
{
    esp_err_t ret = ESP_OK;
    uint32_t i = 0;
    rgb_t color;
    ESP_LOGI(TAG, "*** ILI9488 Test Start ***");
    ret = display_init();
    ESP_ERROR_CHECK(ret);
    display_backlight_set(true);
    for(i = 0; i < (CONFIG_DISPLAY_V_RES / CONFIG_DISPLAY_PARALLEL_LINE_COUNT); i++) {
        switch(i%5) {
            case 0:
                /* red */
                color.red = 0x1F;
                color.green = 0x00;
                color.blue = 0x00;
                break;
            case 1:
                /* green */
                color.red = 0x00;
                color.green = 0x3F;
                color.blue = 0x00;
                break;
            case 2:
                /* blue */
                color.red = 0x00;
                color.green = 0x00;
                color.blue = 0x1F;
                break;
            case 3:
                /* black */
                color.red = 0x00;
                color.green = 0x00;
                color.blue = 0x00;
                break;
            case 4:
                /* white */
                color.red = 0x1F;
                color.green = 0x3F;
                color.blue = 0x1F;
                break;
            default:
                break;
        }
        setBufColor(color);
        display_write_pixel(0, (CONFIG_DISPLAY_H_RES - 1),
                (i * CONFIG_DISPLAY_PARALLEL_LINE_COUNT), 
                (((i + 1) * CONFIG_DISPLAY_PARALLEL_LINE_COUNT) - 1), testColorBuf, TEST_BUF_SIZE);
    }
    ESP_LOGI(TAG, "*** ILI9488 Test End ***");
}