# Summary

This is a firmware source code for the air pollution monitoring sensor. It's designed to run on ESP8266 modules. It polls a DHT22 sensor on GPIO16, PMS7003 on hardware UART and a BMP180 over the I2C interface.

# Building

1. To build this project you need a fully operational xtensa toolchain, and a fully set up [ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK) v3.2. The SDK page describes the steps necessary to achieve that. Do not forget to create a python2 environment and install SDK requirements into it.
2. Acquire Thing's public `certificate.pem.crt` and private `private.pem.key` certificates from AWS IoT Core Console and put them into `main/certs`.
3. Adjust parameters using `make menuconfig`, if necessary.
4. Once everything is set up it's just a matter of:
```
# Build and flash the firmware
$ make flash

# Show firmware debug log.
$ make monitor
```

# License

The code is licensed under MIT unless stated otherwise in a file.
