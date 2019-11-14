
#ifndef _TIME_H_INCLUDED_
#define _TIME_H_INCLUDED_


#include <freertos/FreeRTOS.h>


#define TICKS_FROM_MS(ms)   (((ms) % portTICK_RATE_MS) == 0 ?  \
                              (ms) / portTICK_RATE_MS :        \
                              (ms) / portTICK_RATE_MS + 1)


#endif /* _TIME_H_INCLUDED_ */
