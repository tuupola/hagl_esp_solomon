/*

MIT License

Copyright (c) 2021 Mika Tuupola

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

#ifndef _SSD1351_H
#define _SSD1351_H

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1351_SET_COLUMN_ADDRESS                  (0x15)
#define SSD1351_SET_ROW_ADDRESS                     (0x75)
#define SSD1351_WRITE_RAM_COMMAND                   (0x5C)
#define SSD1351_READ_RAM_COMMAND                    (0x5D)
#define SSD1351_SET_REMAP_DUAL_COM_LINE             (0xA0)
#define SSD1351_SET_DISPLAY_START_LINE              (0xA1)
#define SSD1351_SET_DISPLAY_OFFSET                  (0xA2)
#define SSD1351_SET_DISPLAY_ALL_OFF                 (0xA4)
#define SSD1351_SET_DISPLAY_ALL_ON                  (0xA5)
#define SSD1351_SET_DISPLAY_NORMAL                  (0xA6)
#define SSD1351_SET_DISPLAY_INVERSE                 (0xA7)
#define SSD1351_SET_FUNCTION_SELECTION              (0xAB)
#define SSD1351_SET_SLEEP_MODE_ON                   (0xAE)
#define SSD1351_SET_SLEEP_MODE_OFF                  (0xAF)
#define SSD1351_SET_PHASE_LENGTH                    (0xB1)
#define SSD1351_DISPLAY_ENHANCEMENT                 (0xB2)
#define SSD1351_SET_FRONT_CLOCK_DIVIDER_OSICLLATOR  (0xB3)
#define SSD1351_SET_SEGMENT_LOW_VOLTAGE             (0xB4)
#define SSD1351_SET_GPIO                            (0xB5)
#define SSD1351_SET_SECOND_PRECHARGE_PERIOD         (0xB6)
#define SSD1351_LOOK_UP_TABLE_FOR_GRAY_SCALE        (0xB8)
#define SSD1351_USE_BUILTIN_LINEAR_LUT              (0xB9)
#define SSD1351_SET_PRECHARGE_VOLTAGE               (0xBB)
#define SSD1351_SET_VCOMH_VOLTAGE                   (0xBE)
#define SSD1351_SET_CONTRAST_CURRENT                (0xC1)
#define SSD1351_MASTER_CONTRAST_CURRENT_CONTROL     (0xC7)
#define SSD1351_SET_MULTIPLEX_RATIO                 (0xCA)
#define SSD1351_SET_COMMAND_LOCK                    (0xFD)
#define SSD1351_HORIZONTAL_SCROLL                   (0x96)
#define SSD1351_STOP_MOVING                         (0x9E)
#define SSD1351_START_MOVING                        (0x9F)

/* Entering FDh 12h (A[2]=0b) can unlock the OLED driver IC. That means the     */
/* driver IC resume from the “Lock” state. And the driver IC will then respond  */
/* to the command and memory access. */
//#define SSD1351_SET_COMMAND_LOCK_UNLOCK             (0x12) /* 0b00010010 */
#define SSD1351_SET_COMMAND_LOCK_UNLOCK             (0xb1)

/* For SSD1351_SET_REMAP_DUAL_COM_LINE. */
#define SD1351_HORIZONTAL_ADDRESS_INCREMENT         (0b00000000) /* 0x00 */
#define SD1351_VERTICAL_ADDRESS_INCREMENT           (0b00000001) /* Swap XY 0x01 */
#define SD1351_COLUMN_ADDRESS_REMAP                 (0b00000010) /* Mirror X 0x02 */
#define SD1351_COLOR_REMAP                          (0b00000100) /* RGB to BGR 0x04 */
#define SD1351_COM_SCAN_DIRECTION_REMAP             (0b00001000) /* 0x08 */
#define SD1351_COM_ODD_EVEN_SPLIT                   (0b00010000) /* Mirror Y 0x10 */
#define SD1351_COLOR_MODE_65K                       (0b00100000) /* RGB565 0x20 */
#define SD1351_COLOR_MODE_262K                      (0b01000000) /* RGB666 0x40 */
#define SD1351_COLOR_MODE_262K_FORMAT2              (0b01100000) /* RGB666 format 2 */

#ifdef __cplusplus
}
#endif
#endif /* _SSD1351_H */
