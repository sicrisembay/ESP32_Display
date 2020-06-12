
#include "sdkconfig.h"
#if defined(CONFIG_ESP32_DISPLAY)

#include "display.h"

esp_err_t display_init(void)
{
    return ESP_OK;
}

#endif /* CONFIG_ESP32_DISPLAY */