#ifndef DISPLAY_SPI_INTERFACE_H
#define DISPLAY_SPI_INTERFACE_H

#include "esp_err.h"
#include "../include/display.h"

esp_err_t display_interface_init(void);
esp_err_t display_interface_write_px(const uint8_t *pBuf, const uint32_t nPixel);
esp_err_t display_interface_write_command(const uint8_t cmd, const uint8_t *pBuf, const uint32_t nBuf);

#endif /* DISPLAY_SPI_INTERFACE_H */