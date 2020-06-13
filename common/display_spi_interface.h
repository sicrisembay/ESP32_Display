#ifndef DISPLAY_SPI_INTERFACE_H
#define DISPLAY_SPI_INTERFACE_H

#include "esp_err.h"
#include "../include/display.h"

esp_err_t display_interface_init(void);
esp_err_t display_interface_write_px(rgb_t *pBuf, uint32_t nPixel);
esp_err_t display_interface_write_command(uint8_t cmd, uint8_t *pBuf, uint32_t nBuf);

#endif /* DISPLAY_SPI_INTERFACE_H */