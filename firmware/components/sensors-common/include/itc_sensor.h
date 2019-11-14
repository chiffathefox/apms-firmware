
#ifndef _ITC_SENSOR_H_INCLUDED_
#define _ITC_SENSOR_H_INCLUDED_


#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <esp_err.h>


enum itc_sensor_type {
    ITC_SENSOR_TYPE_TAP = 0,  /* Temperature (0.1*C) and pressure (Pa). */
    ITC_SENSOR_TYPE_TAH       /* Temperature (0.1*C) and humidity (0.1%). */
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


#define itc_sensor_update_tap(u)                                              \
    (struct itc_sensor_update_tap *) ((unsigned char *) (u) -                 \
            offsetof(struct itc_sensor_update_tap, upd))


#define itc_sensor_update_tah(u)                                              \
    (struct itc_sensor_update_tah *) ((unsigned char *) (u) -                 \
            offsetof(struct itc_sensor_update_tah, upd))


static inline void
itc_sensor_send_fn(QueueHandle_t queue, struct itc_sensor_update *update)
{
    assert(xQueueSendToBack(queue, &update, 0) == pdPASS);
}


#define itc_sensor_send(q, update)                                            \
    itc_sensor_send_fn(q, &(update)->upd);


#define ITC_SENSOR_TASK_PRIO  (configMAX_PRIORITIES / 2)


#endif /* _ITC_SENSOR_H_INCLUDED_ */
