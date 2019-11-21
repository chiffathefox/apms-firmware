
#ifndef _PMS_H_INCLUDED_
#define _PMS_H_INCLUDED_


#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <driver/uart.h>

#include "itc_sensor.h"


/* 
 * PMS part code consists of the following parts:
 * PMS[AA][BB]
 *
 * Where AA is a model number (enum pms_model) and
 *       BB is the minimum distinguishable particle diameter (enum pms_mdpd).
 */

enum pms_model {
    PMS_MODEL_70 = 0
};

enum pms_mdpd {
    PMS_MDPD_03 = 0,  /* 0.3 um */
    PMS_MDPD_05,      /* 0.5 um */
    PMS_MDPD_10,      /* 1.0 um */
    PMS_MDPD_25,      /* 2.5 um */
    PMS_MDPD_MAX
};

enum pms_cnvmode {

    /* Make measurements as quickly as possible. */

    PMS_CNVMODE_FAST = 0,

    /* 
     * Keep the sensor off most of the time.
     * Please, note that measurements may take considerably longer
     * (up to 2 * PMS_SPINUP_TICKS).
     */

    PMS_CNVMODE_LP,
    PMS_CNVMODE_MAX
};

struct pms {
    uart_port_t          port;
    enum pms_model       model;
    enum pms_mdpd        mdpd;
    enum pms_cnvmode     cnvmode;

    QueueHandle_t        trigq;
    TaskHandle_t         task;
    TickType_t           last_chpwr;
};

struct pms_trig_conv {
    TickType_t                      keep_alive;
    QueueHandle_t                   updatesq;

    struct itc_sensor_update_pm     update;
};


/**
 * @brief Initialize a PMS sensor.
 *
 * For each sensor a new task and a trigger queue is created. Which makes
 * `pms_trig_conv()' calls thread-safe.
 *
 * @param pms Device handle.
 * @param port UART port the sensor is connected to.
 * @param model Model of the sensor.
 * @param mdpd Minimum ditinguishable particle diamete of a sensor.
 * @param strategy Measurements conversion strategy.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Hardware or software issue. `pms_trig_conv()' calls may still
 *                work.
 */

esp_err_t pms_init(struct pms *pms, uart_port_t port, enum pms_model model,
        enum pms_mdpd mdpd, enum pms_cnvmode cnvmode);

/**
 * @brief Trigger a conversion.
 *
 * If a conversion is still running and trigger queue is full, then this
 * function will wait `timeout' ticks to send a message to a queue.
 *
 * @param pms Device handle.
 * @param params Conversion parameters.
 * @param timeout `xQueueSendToBack' timeout.
 * @return
 *     - pdTRUE Seccessfully pushed a message.
 *     - pdFALSE Trigger queue is full.
 */

BaseType_t pms_trig_conv(struct pms *pms, struct pms_trig_conv *params,
        TickType_t timeout);


#endif /* _PMS_H_INCLUDED_ */
