
#ifndef _ITC_SENSOR_H_INCLUDED_
#define _ITC_SENSOR_H_INCLUDED_


#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <esp_err.h>


#define ITC_SENSOR_INVTEMP      (-0xFFFF)
#define ITC_SENSOR_INVPRESSURE  (-0xFFFFFFFF)
#define ITC_SENSOR_INVHUMIDITY  (-1)
#define ITC_SENSOR_INVPM        (-1)


enum itc_sensor_type {
    ITC_SENSOR_TYPE_TAP = 0,  /* Temperature (0.1*C) and pressure (Pa). */
    ITC_SENSOR_TYPE_TAH,      /* Temperature (0.1*C) and humidity (0.1%). */
    ITC_SENSOR_TYPE_PM        /* Particulate matter concentration (ug/m3). */
};

struct itc_sensor_update {
    esp_err_t                status;
    enum itc_sensor_type     type;
};

struct itc_sensor_update_tap {
    struct itc_sensor_update     upd;
    int                          temp;
    long                         pressure;
};

struct itc_sensor_update_tah {
    struct itc_sensor_update     upd;
    int                          temp;
    int                          humidity;
};

struct itc_sensor_update_pm {
    struct itc_sensor_update     upd;
    short                        pm1d0;  /* PM1.0 concentration (ug/m^3). */
    short                        pm2d5;  /* PM2.5 concentration (ug/m^3). */
    short                        pm10d;  /* PM10 concentration (ug/m^3). */
};


#define itc_sensor_update_tap(u)                                              \
    (struct itc_sensor_update_tap *) ((unsigned char *) (u) -                 \
            offsetof(struct itc_sensor_update_tap, upd))


#define itc_sensor_update_tah(u)                                              \
    (struct itc_sensor_update_tah *) ((unsigned char *) (u) -                 \
            offsetof(struct itc_sensor_update_tah, upd))


#define itc_sensor_update_pm(u)                                               \
    (struct itc_sensor_update_pm *) ((unsigned char *) (u) -                  \
            offsetof(struct itc_sensor_update_pm, upd))


static inline void
itc_sensor_send_fn(QueueHandle_t queue, struct itc_sensor_update *update)
{
    assert(xQueueSendToBack(queue, &update, 0) == pdPASS);
}


#define itc_sensor_send(q, update)                                            \
    itc_sensor_send_fn(q, &(update)->upd);


#define ITC_SENSOR_TASK_PRIO  (configMAX_PRIORITIES / 2)


/*
 * TODO: some of the code for sensor API repeats. Make those parts shared.
 */


#endif /* _ITC_SENSOR_H_INCLUDED_ */
