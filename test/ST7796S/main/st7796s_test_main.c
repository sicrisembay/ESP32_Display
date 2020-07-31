#include "sdkconfig.h"
#include "esp_log.h"
#include "../../../include/display.h"

static const char *TAG = "test_main";

#define N_PIXEL_IN_BUFF  (CONFIG_DISPLAY_H_RES * CONFIG_DISPLAY_PARALLEL_LINE_COUNT)
static uint8_t testColorBuf[N_PIXEL_IN_BUFF * 2];

static void setBufColor(rgb_t color)
{
    int i = 0;
    for(i = 0; i < (N_PIXEL_IN_BUFF * 2); i = i + 2) {
#if 0
        testColorBuf[i] = (uint8_t)(((color.red & 0x1F) << 3) |
                                ((color.green & 0x3F) >> 3));
        testColorBuf[i+1] = (uint8_t)((((color.green & 0x3F) << 5) |
                                (color.blue & 0x1F)));
#else
        testColorBuf[i] = (uint8_t)(color.raw >> 8);
        testColorBuf[i+1] = (uint8_t)(color.raw & 0xFF);
#endif
    }
}

void app_main()
{
    esp_err_t ret = ESP_OK;
    uint32_t i = 0;
    rgb_t color;

    ESP_LOGI(TAG, "*** ST7796S Test Start ***");
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
                (((i + 1) * CONFIG_DISPLAY_PARALLEL_LINE_COUNT) - 1), testColorBuf, N_PIXEL_IN_BUFF);
    }

    ESP_LOGI(TAG, "*** ST7796S Test End ***");
}