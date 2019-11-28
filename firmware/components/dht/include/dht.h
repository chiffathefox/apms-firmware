
#ifndef _DHT_H_INCLUDED_
#define _DHT_H_INCLUDED_


#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <driver/gpio.h>

#include "itc_sensor.h"


enum dht_type {
    DHT_TYPE_AM23xx = 0
};

struct dht {
    gpio_num_t                       pin;
    enum dht_type                    type;

    QueueHandle_t                    trigq;
    TaskHandle_t                     task;
    TickType_t                       last_conv;
};

struct dht_trig_conv {

    /* Use results from conversions made no earlier than `keep_alive'. */

    TickType_t                       keep_alive;
    QueueHandle_t                    updatesq;
    struct itc_sensor_update_tah     update;
};


/**
 * @brief Initialize a DHT sensor.
 *
 * For each sensor a new task and a trigger queue is created. Which makes
 * `dht_trig_conv()' calls thread-safe.
 *
 * @param dht Device handle.
 * @param pin GPIO pin the sensor is connected to.
 * @param type Type of the sensor.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Hardware or software issue. `dht_trig_conv()' calls may still
 *                work.
 */

esp_err_t dht_init(struct dht *dht, gpio_num_t pin, enum dht_type type);

/**
 * @brief Trigger a conversion.
 *
 * If a conversion is still running and trigger queue is full, then this
 * function will wait `timeout' ticks to send a message to a queue.
 *
 * @param dht Device handle.
 * @param params Conversion parameters.
 * @param timeout `xQueueSendToBack' timeout.
 * @return
 *     - pdTRUE Seccessfully pushed a message.
 *     - pdFALSE Trigger queue is full.
 */

BaseType_t dht_trig_conv(struct dht *dht, struct dht_trig_conv *params,
        TickType_t timeout);


#endif /* _DHT_H_INCLUDED_ */
