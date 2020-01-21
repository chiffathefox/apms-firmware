# Summary

This is a firmware source code for the air pollution monitoring sensor. It's designed to run on ESP8266 modules. It polls a DHT22 sensor on GPIO16, PMS7003 on hardware UART and a BMP180 over the I2C interface.

# Building

To build this project you need a fully operational xtensa toolchain, and a fully set up [ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK). The SDK page describes the steps necessary to achieve that. Do not forget to create a python environment and install SDK requirements into it. Once it is all set up it's just a matter of:
```
# Build and flash the firmware
$ make flash

# Show firmware debug log.
$ make monitor
```

# License

The code is licensed under MIT unless stated otherwise in a file.
