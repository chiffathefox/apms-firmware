
#ifndef _TIME_H_INCLUDED_
#define _TIME_H_INCLUDED_


#include <freertos/FreeRTOS.h>


#define TICKS_FROM_MS(ms)   (((ms) % portTICK_RATE_MS) == 0 ?  \
                              (ms) / portTICK_RATE_MS :        \
                              (ms) / portTICK_RATE_MS + 1)


#define ticks_to_ms(tick)   (tick * portTICK_RATE_MS)


/**
 * @brief Delay until `*last_wakeup + period' and fill `last_wakeup' with
 *        current ticks count value.
 *
 * This function is different to `vTaskDelayUntil()' because it takes into
 * consideration the possibility that `*last_wakeup + period' value may be in the
 * past.
 *
 * @param last_wakeup Ticks count the task last woke up at.
 * @param period Minimal period the task should be executed at.
 */

static inline void
ticks_delay_until(TickType_t *last_wakeup, TickType_t period)
{
    if (*last_wakeup != 0) {
        vTaskDelayUntil(last_wakeup, period);
    }

    *last_wakeup = xTaskGetTickCount();
}


#endif /* _TIME_H_INCLUDED_ */
