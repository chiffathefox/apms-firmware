
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_err.h>
#include <rom/ets_sys.h>
#include <driver/gpio.h>

#include "ticks.h"
#include "itc_sensor.h"
#include "bytes.h"

#include "dht.h"


#define DHT_TRIGQ_LENGTH    5
#define DHT_START_TIME      18   /* ms */
#define DHT_BLOCK_INTERVAL  2    /* us */
#define DHT_DATA_BITS       40
#define DHT_DATA_BYTES      (DHT_DATA_BITS / 8)


#define DHT_CHECK_LOGW(x, ...)                                                 \
    if ((x) != ESP_OK) {                                                       \
        ESP_LOGW(TAG, __VA_ARGS__);                                            \
                                                                               \
        return ESP_FAIL;                                                       \
    }


static const char    *TAG = "dht";


static void dht_task(void *param);
static inline void dht_fill_update_fail(struct itc_sensor_update_tah *update);
static inline void dht_fill_update_success(struct itc_sensor_update_tah *update,
        int temp, int humidity);
static esp_err_t dht_block_gpio(gpio_num_t pin, uint32_t timeout, int level,
        uint32_t *duration);
static inline void dht_send_start(gpio_num_t pin);
static inline esp_err_t dht_read(gpio_num_t pin,
        unsigned char data[DHT_DATA_BYTES]);
static esp_err_t dht_conv(struct dht *dht, int *temp, int *humidity);


/*
 * TODO: DHT11 support.
 */


esp_err_t
dht_init(struct dht *dht, gpio_num_t pin, enum dht_type type)
{
    gpio_config_t     conf;
    BaseType_t        rc;

    dht->pin = pin;
    dht->type = type;
    dht->task = NULL;

    conf.pin_bit_mask = BIT(pin);
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;

    assert(gpio_config(&conf) == ESP_OK);

    /* Check if the pullup resistor was installed. */

    if (gpio_get_level(pin) == 0) {
        ESP_LOGW(TAG, "no pullup resistor detected on pin %d", pin);

        return ESP_FAIL;
    }

    dht->trigq = xQueueCreate(DHT_TRIGQ_LENGTH,
            sizeof (struct dht_trig_conv *));

    if (dht->trigq == NULL) {
        ESP_LOGE(TAG, "xQueueCreate failed");

        return ESP_FAIL;
    }

    rc = xTaskCreate(dht_task, "dht", 1000, dht, ITC_SENSOR_TASK_PRIO,
            &dht->task);

    if (rc != pdPASS) {

        /* 
         * The only cause of this may be insufficient memory,
         * but we may still recover from this error by sending update failures
         * to `updates' queue.
         */

        dht->task = NULL;

        ESP_LOGW(TAG, "xTaskCreate failed");

        return ESP_FAIL;
    }

    return ESP_OK;
}


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

BaseType_t
dht_trig_conv(struct dht *dht, struct dht_trig_conv *params, TickType_t timeout)
{
    if (dht->task == NULL) {
        dht_fill_update_fail(&params->update);
        itc_sensor_send(params->updatesq, &params->update);

        return pdTRUE;
    }

    return xQueueSendToBack(dht->trigq, &params, timeout);
}


/**
 * @brief DHT task that handles updates from a trigger queue.
 *
 * @param param DHT device handle.
 */

static void
dht_task(void *param)
{
    struct dht              *dht;
    struct dht_trig_conv    *params;
    int                      temp, humidity, first;
    TickType_t               last_tick, tick;
    esp_err_t                err;

    dht = param;
    first = 1;
    last_tick = xTaskGetTickCount();

    for (;;) {
        xQueueReceive(dht->trigq, &params, portMAX_DELAY);

        tick = xTaskGetTickCount();

        if (!first && tick - last_tick < params->keep_alive) {
            dht_fill_update_success(&params->update, temp, humidity);
        } else {
            err = dht_conv(dht, &temp, &humidity);

            if (err == ESP_OK) {
                first = 0;
                last_tick = tick;

                dht_fill_update_success(&params->update, temp, humidity);
            } else {
                dht_fill_update_fail(&params->update);
            }
        }

        itc_sensor_send(params->updatesq, &params->update);
    }
}


/**
 * @brief Fill an update structure with data from a failed conversion.
 * 
 * @param update Structure to fill.
 */

static inline void
dht_fill_update_fail(struct itc_sensor_update_tah *update)
{
    update->upd.status = ESP_FAIL;
    update->upd.type = ITC_SENSOR_TYPE_TAH;
}


/**
 * @brief Fill an update structure from a successful conversion.
 *
 * @param update Structure to fill.
 * @param temp Temperature.
 * @param humidity Humidity.
 */

static inline void
dht_fill_update_success(struct itc_sensor_update_tah *update, int temp,
        int humidity)
{
    update->upd.status = ESP_OK;
    update->upd.type = ITC_SENSOR_TYPE_TAH;
    update->temp = temp;
    update->humidity = humidity;
}


/**
 * @brief Wait for a GPIO pin state.
 *
 * Waits up to `timeout' microseconds until the state of `pin' becomes `level'.
 *
 * @param pin GPIO pin.
 * @param timeout Maximum number of microsecond to wait for the specified level.
 * @param level GPIO state to wait for.
 * @param duration Number of microsecond spent waiting destination variable.
 */

static esp_err_t
dht_block_gpio(gpio_num_t pin, uint32_t timeout, int level, uint32_t *duration)
{
    uint32_t     i;

    ets_delay_us(DHT_BLOCK_INTERVAL);

    for (i = DHT_BLOCK_INTERVAL; i < timeout; i += DHT_BLOCK_INTERVAL) {
        if (gpio_get_level(pin) == level) {
            if (duration != NULL) {
                *duration = i;
            }

            return ESP_OK;
        }

        ets_delay_us(DHT_BLOCK_INTERVAL);
    }

    return ESP_ERR_TIMEOUT;
}


/**
 * @brief Send a start signal to the DHT sensor.
 *
 * @param pin The GPIO pin the device is at.
 */

static inline void
dht_send_start(gpio_num_t pin)
{

    /* 
     * The line has an external pullup. That is why we use the open-darin mode.
     */
    
    assert(gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD) == ESP_OK);
    assert(gpio_set_level(pin, 0) == ESP_OK);
    vTaskDelay(TICKS_FROM_MS(DHT_START_TIME));
    assert(gpio_set_direction(pin, GPIO_MODE_INPUT) == ESP_OK);
}


/**
 * @brief Read data from DHT.
 *
 * This function may only be called after a start signal has been sent
 * by the MCU.
 *
 * @param pin The GPIO pin the device is at.
 * @param data Data buffer to write to.
 * @return
 *     - ESP_OK Success.
 *     - ESP_FAIL Protocol error.
 */

static inline esp_err_t
dht_read(gpio_num_t pin, unsigned char data[DHT_DATA_BYTES])
{
    int               i;
    uint32_t          low_duration, high_duration;

    DHT_CHECK_LOGW(dht_block_gpio(pin, 40, 0, NULL), "ACK low not received");
    DHT_CHECK_LOGW(dht_block_gpio(pin, 80, 1, NULL), "ACK high not received");
    DHT_CHECK_LOGW(dht_block_gpio(pin, 80, 0, NULL), "bit start not received");

    memset(data, 0, DHT_DATA_BYTES);

    for (i = 0; i < DHT_DATA_BITS; i++) {
        DHT_CHECK_LOGW(dht_block_gpio(pin, 50, 1, &low_duration),
                "timed out waiting for a data bit");

        DHT_CHECK_LOGW(dht_block_gpio(pin, 70, 0, &high_duration),
                "timed out waiting for the end of a data bit");

        data[i / 8] |= (high_duration > low_duration) << (7 - (i % 8));
    }

    return ESP_OK;
}


/**
 * Trigger temperature and pressure conversion and fetch the data from the
 * sensor.
 * This function may waste up to 5ms of CPU time.
 *
 * @param dht Device handle.
 * @param temp Temperature destination variable (0.1*C).
 * @param pressure Pressure destination variable (Pa).
 * @return
 *     - ESP_OK Success.
 */

static esp_err_t
dht_conv(struct dht *dht, int *temp, int *humidity)
{
    esp_err_t         err;
    unsigned char     data[DHT_DATA_BYTES], checksum;

    assert(dht->type == DHT_TYPE_AM23xx);

    dht_send_start(dht->pin);

    taskENTER_CRITICAL();
    err = dht_read(dht->pin, data);
    taskEXIT_CRITICAL();

    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    /* 
     * The sensor sends out 5 bytes of data:
     * [rh_msb][rh_lsb][temp_msb][temp_lsb][checksum]
     * Where checksum = rh_msb + rh_lsb + temp_msb + temp_lsb
     */

    checksum = data[0] + data[1] + data[2] + data[3];

    if (checksum != data[4]) {
        ESP_LOGW(TAG, "checksum failed (%02x != %02x)", checksum, data[4]);

        return ESP_FAIL;
    }

    *humidity = bytes_be_to_u16(data);
    *temp = bytes_be_to_u16(data + 2);

    return ESP_OK;
}

