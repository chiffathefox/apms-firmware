
#ifndef _MDBG_H_INCLUDED_
#define _MDBG_H_INCLUDED_


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


static inline void
mdbg_info_fn(int line)
{
    ESP_LOGD(pcTaskGetTaskName(NULL),
            "meminfo:%d: free heap %d, free stack %lu", 
            line, heap_caps_get_minimum_free_size(0),
            uxTaskGetStackHighWaterMark(NULL));
}


#define mdbg_info()  mdbg_info_fn(__LINE__)


#endif /* _MDBG_H_INCLUDED_ */
