
#ifndef _BMP180_H_INCLUDED_
#define _BMP180_H_INCLUDED_


#include <freertos/queue.h>

#include <esp_err.h>

#include <driver/i2c.h>

#include "itc_sensor.h"


/* BMP180 oversampling ratio. */

enum bmp180_oss {
    BMP180_OSS_1 = 0,   /* 1 sample  */
    BMP180_OSS_2,       /* 2 samples */
    BMP180_OSS_4,       /* 4 samples */
    BMP180_OSS_8        /* 8 samples */
};

struct bmp180_trig_conv {

    /* Conversion parameters. */

    enum bmp180_oss                  oss;
    
    /* 
     * Target update structure. 
     * Must be alive up to the moment an update is received.
     */

    struct itc_sensor_update_tap     update;
};


esp_err_t bmp180_init(i2c_port_t port, QueueHandle_t updates);
void bmp180_trig_conv(struct bmp180_trig_conv *params);


#endif /* _BMP180_H_INCLUDED_ */
