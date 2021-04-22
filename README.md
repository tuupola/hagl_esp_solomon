
# Solomon HAL for HAGL Graphics Library

HAL for HAGL graphics library for [Solomon Systech](https://www.solomon-systech.com/) based displays. Since they do not follow the MIPI DCS standard each SSDxxxx chip needs an ad hoc driver. Currently only SSD1351 is supported.  This code is still considered work in progress.

[![Software License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](LICENSE.md)

## Usage

To use with an ESP-IDF project you include this HAL and the [HAGL graphics library](https://github.com/tuupola/hagl) itself.  If you are using CMake based build the HAL **must** be in folder named `hagl_hal`.

```
$ cd components
$ git submodule add git@github.com:tuupola/hagl_esp_solomon.git hagl_hal
$ git submodule add git@github.com:tuupola/hagl.git
```

You can alter display behaviour via `menuconfig`. If you choose to use back buffer all drawing operations will be fast. Downside is that back buffer requires lot of memory. To reduce flickering you can also choose to lock back buffer while flushing. Locking will slow down draw operations though.

```
$ idf.py menuconfig
```

You can also use the older GNU Make based build system.

```
$ make menuconfig
```

For example usage see [ESP effects Solomon branch](https://github.com/tuupola/esp_effects/tree/solomon).

## License

The MIT License (MIT). Please see [License File](LICENSE) for more information.
