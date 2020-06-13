#ifndef ESP32_DISPLAY_H
#define ESP32_DISPLAY_H

#include "sdkconfig.h"
#if defined(CONFIG_ESP32_DISPLAY)

#include "esp_err.h"

typedef struct {
    union {
#if defined(CONFIG_DISPLAY_COLOR_RGB565)
    uint16_t raw;
    struct {
        uint16_t blue   : 5;
        uint16_t green  : 6;
        uint16_t red    : 5;
    };
#elif defined(CONFIG_DISPLAY_COLOR_ARGB8888)
    uint32_t raw;
    struct {
        uint32_t blue   : 8;
        uint32_t green  : 8;
        uint32_t red    : 8;
        uint32_t alpha  : 8;
    };
#else
#error "Unsupported RGB Format!"
#endif
    };
} rgb_t;

esp_err_t display_init(void);
esp_err_t display_write_pixel(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye, rgb_t *pBuf, uint32_t nPixel);
esp_err_t display_backlight_set(bool state);
#endif /* CONFIG_ESP32_DISPLAY */

#endif /* ESP32_DISPLAY_H */