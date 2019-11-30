
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <esp_err.h>

#include <driver/i2c.h>

#include "dht.h"
#include "pms.h"
#include "bmp180.h"
#include "ticks.h"
#include "itc_sensor.h"


#define APP_I2C_PORT          I2C_NUM_0
#define APP_SENSORS_COUNT     3

/* For code sanity checks only. */

#define APP_QUEUE_WDT_TICKS  TICKS_FROM_MS(300000)


#define APP_FAST_MODE        0


#if (APP_FAST_MODE)


#    define APP_PMS_CNVMODE  PMS_CNVMODE_FAST
#    define APP_MEAS_DELAY   0


#else


#    define APP_PMS_CNVMODE  PMS_CNVMODE_LP
#    define APP_MEAS_DELAY   30000


#endif


struct app_data {
    int       temp;         /* The most accurate available temperature. */
    int       temp_coarse;
    long      pressure;
    int       humidity;
    short     pm1d0;
    short     pm2d5;
    short     pm10d;
};


struct app_conf {
    TickType_t     pms_safety_delay;
    TickType_t     meas_delay;
    TickType_t     max_meas_delay;
};


#define app_data_log(data, fn)                                                  \
    fn(TAG, "data: temp %d 0.1*C; temp_coarse %d 0.1*C; "                       \
            "pressure %ld Pa; humidity %d 0.1%%RH; "                            \
            "PM1.0 %d ug/m^3; PM2.5 %d ug/m^3; PM10 %d ug/m^3",                 \
            (data)->temp, (data)->temp_coarse, (data)->pressure,                \
            (data)->humidity, (data)->pm1d0, (data)->pm2d5, (data)->pm10d);


static esp_err_t app_i2c_master_init(i2c_port_t port);
static inline void app_try(const char *name, esp_err_t err);
static void app_main_task(void *param);
static inline void app_data_init(struct app_data *data);
static inline int app_data_is_ok(struct app_data *data);
static void app_pop_update(struct app_data *data);
static void app_conv_weather_data(struct app_data *data);
static void app_conf_load(struct app_conf *conf);


static const char           *TAG = "main";
static QueueHandle_t         app_sensors_updates;
struct dht                   app_dht;
struct pms                   app_pms;
struct dht_trig_conv         app_dht_params;
struct pms_trig_conv         app_pms_params;
struct bmp180_trig_conv      app_bmp180_params;
struct app_conf              app_conf;


void
app_main()
{
    esp_err_t      err;
    BaseType_t     rc;

    ESP_LOGI(TAG, "Init stage");

    app_sensors_updates = xQueueCreate(APP_SENSORS_COUNT,
            sizeof (struct itc_sensor_update *));

    assert(app_sensors_updates != NULL);

    err = app_i2c_master_init(APP_I2C_PORT);

    if (err == ESP_OK) {
        err = bmp180_init(APP_I2C_PORT, app_sensors_updates);

        assert(err != ESP_ERR_NO_MEM);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "bmp180_init failed (%d)", err);
        }
    } else {

        /* Sensors with I2C interface cannot be intialized. */

        ESP_LOGW(TAG, "app_i2c_master_init failed (%d)", err);
    }

    app_try("dht_init", dht_init(&app_dht, GPIO_NUM_16, DHT_TYPE_AM23xx));
    app_try("pms_init", pms_init(&app_pms, UART_NUM_0, PMS_MODEL_70, PMS_MDPD_03,
                APP_PMS_CNVMODE));

    /* Initizalize trigger structures. */

    app_bmp180_params.oss = BMP180_OSS_1;

    app_dht_params.keep_alive = 0;
    app_dht_params.updatesq = app_sensors_updates;

    app_pms_params.keep_alive = 0;
    app_pms_params.updatesq = app_sensors_updates;

    app_conf_load(&app_conf);

    rc = xTaskCreate(app_main_task, "app_main_task", 8000, NULL,
            ITC_SENSOR_TASK_PRIO - 1, NULL);

    assert(rc == pdTRUE);

    ESP_LOGI(TAG, "Init stage finished");
}


/**
 * Configure I2C interface to be in MASTER mode with
 * internal pull-up resistors DISABLED.
 *
 * @param port I2C master port.
 * @return
 *     - ESP_OK               Success
 *     - ESP_FAIL             Driver install error.
 *     - ESP_ERR_INVALID_ARG  Parameter error.
 */

static esp_err_t
app_i2c_master_init(i2c_port_t port)
{
    esp_err_t        err;
    i2c_config_t     conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_4;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = GPIO_NUM_5;
    conf.scl_pullup_en = 0;
    conf.clk_stretch_tick = 300;

    err = i2c_driver_install(port, conf.mode);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_driver_install failed (%d)", err);

        return err;
    }

    err = i2c_param_config(port, &conf);

    return err;
}


static inline void
app_try(const char *name, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s() failed (%d)", name, err);
    }
}


static void
app_main_task(void *param)
{
    int                          rc;
    TickType_t                   delay;
    struct app_data              data;
    struct itc_sensor_update    *update;

    delay = app_conf.meas_delay;

    for (;;) {
        ESP_LOGD(TAG, "triggerring conversions ...");

        app_conv_weather_data(&data);

        /*
         * If `data.temp' == ITC_SENSOR_INVTEMP,
         * then this fuction returns false.
         */

        /* TODO: take a look at oss */

        rc = pms_is_safe(&app_pms, data.temp, data.humidity, 0);

        if (rc) {
            assert(pms_trig_conv(&app_pms, &app_pms_params, 0) == pdTRUE);
            assert(xQueuePeek(app_sensors_updates, &update,
                        APP_QUEUE_WDT_TICKS) == pdTRUE);

            app_conv_weather_data(&data);
            app_pop_update(&data);
        } else {
            ESP_LOGW(TAG, "operating conditions are not safe for the PMS");

            if (data.temp != ITC_SENSOR_INVTEMP && 
                    data.humidity != ITC_SENSOR_INVHUMIDITY) {

                /* 
                 * Increase the delay only if the values were
                 * actually measured.
                 */

                delay += app_conf.pms_safety_delay;
            }
        }

        if (app_data_is_ok(&data)) {
            app_data_log(&data, ESP_LOGI);

            rc = pms_is_safe(&app_pms, data.temp, data.humidity, data.pm10d);
            delay = rc ? 
                app_conf.meas_delay : 
                delay + app_conf.pms_safety_delay;
        } else {
            app_data_log(&data, ESP_LOGW);
        }

        assert(xQueueReceive(app_sensors_updates, &update, 0) == pdFALSE);

        if (delay > app_conf.max_meas_delay) {
            delay = app_conf.max_meas_delay;
        }

        vTaskDelay(delay);
    }
}


static inline void
app_data_init(struct app_data *data)
{
    data->temp = ITC_SENSOR_INVTEMP;
    data->temp_coarse = ITC_SENSOR_INVTEMP;
    data->pressure = ITC_SENSOR_INVPRESSURE;
    data->humidity = ITC_SENSOR_INVHUMIDITY;
    data->pm1d0 = data->pm2d5 = data->pm10d = ITC_SENSOR_INVPM;
}


static inline int
app_data_is_ok(struct app_data *data)
{
    return data->temp != ITC_SENSOR_INVTEMP &&
        data->temp_coarse != ITC_SENSOR_INVTEMP &&
        data->pressure != ITC_SENSOR_INVPRESSURE &&
        data->humidity != ITC_SENSOR_INVHUMIDITY &&
        data->pm1d0 != ITC_SENSOR_INVPM &&
        data->pm2d5 != ITC_SENSOR_INVPM &&
        data->pm10d != ITC_SENSOR_INVPM;
}


static void
app_pop_update(struct app_data *data)
{
    struct itc_sensor_update        *update;
    struct itc_sensor_update_tah    *tah;
    struct itc_sensor_update_tap    *tap;
    struct itc_sensor_update_pm     *pm;

    assert(xQueueReceive(app_sensors_updates, &update,
                APP_QUEUE_WDT_TICKS) == pdTRUE);

    if (update->status != ESP_OK) {
        ESP_LOGW(TAG, "sensor %d update failed (%d)",
                update->type, update->status);

        return;
    }

    switch (update->type) {


    case ITC_SENSOR_TYPE_TAP:

        tap = itc_sensor_update_tap(update);
        data->temp_coarse = tap->temp;
        data->pressure = tap->pressure;

        break;


    case ITC_SENSOR_TYPE_TAH:

        tah = itc_sensor_update_tah(update);
        data->temp = tah->temp;
        data->humidity = tah->humidity;

        break;


    case ITC_SENSOR_TYPE_PM:

        pm = itc_sensor_update_pm(update);
        data->pm1d0 = pm->pm1d0;
        data->pm2d5 = pm->pm2d5;
        data->pm10d = pm->pm10d;

        break;


    default:

        ESP_LOGE(TAG, "received an unknown update type (%d)",
                update->type);

        assert(0);

        break;


    }
}


static void
app_conv_weather_data(struct app_data *data)
{
    bmp180_trig_conv(&app_bmp180_params);
    assert(dht_trig_conv(&app_dht, &app_dht_params, 0) == pdTRUE);

    app_data_init(data);

    for (int i = 0; i < 2; i++) {
        app_pop_update(data);
    }

    if (data->temp == ITC_SENSOR_INVPRESSURE) {
        data->temp = data->temp_coarse;

        ESP_LOGW(TAG, "accurate temperature was replaced with a coarse one");
    }
}


static void
app_conf_load(struct app_conf *conf)
{
    conf->pms_safety_delay = TICKS_FROM_MS(60000);
    conf->meas_delay = TICKS_FROM_MS(APP_MEAS_DELAY);
    conf->max_meas_delay = conf->meas_delay * 5;
}
