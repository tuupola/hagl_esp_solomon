/*

MIT License

Copyright (c) 2019-2021 Mika Tuupola

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

-cut-

This file is part of the Solomon HAL for HAGL graphics library:
https://github.com/tuupola/hagl_esp_solomon

SPDX-License-Identifier: MIT

*/

#ifndef _SOLOMON_DISPLAY_H
#define _SOLOMON_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <driver/spi_master.h>

#include "sdkconfig.h"
#include "hagl_hal.h"

// #define DISPLAY_WIDTH   (CONFIG_MIPI_DISPLAY_WIDTH)
// #define DISPLAY_HEIGHT  (CONFIG_MIPI_DISPLAY_HEIGHT)
// #define DISPLAY_DEPTH   (CONFIG_MIPI_DISPLAY_DEPTH)

#define SPI_MAX_TRANSFER_SIZE   (DISPLAY_WIDTH * DISPLAY_HEIGHT * DISPLAY_DEPTH)

#define SOLOMON_DISPLAY_ADDRESS_MODE ( \
    CONFIG_MIPI_DCS_ADDRESS_MODE_MIRROR_Y | \
    CONFIG_MIPI_DCS_ADDRESS_MODE_MIRROR_X | \
    CONFIG_MIPI_DCS_ADDRESS_MODE_SWAP_XY | \
    CONFIG_MIPI_DCS_ADDRESS_MODE_FLIP_X | \
    CONFIG_MIPI_DCS_ADDRESS_MODE_FLIP_Y | \
    CONFIG_MIPI_DCS_ADDRESS_MODE_BGR \
)

void solomon_display_init(spi_device_handle_t *spi);
size_t solomon_display_write(spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint8_t *buffer);
void solomon_display_ioctl(spi_device_handle_t spi, uint8_t command, uint8_t *data, size_t size);
void solomon_display_close(spi_device_handle_t spi);

#ifdef __cplusplus
}
#endif
#endif /* _SOLOMON_DISPLAY_H */