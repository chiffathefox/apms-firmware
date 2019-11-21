
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


#define APP_I2C_PORT       I2C_NUM_0
#define APP_SENSORS_COUNT  3


static esp_err_t app_i2c_master_init(i2c_port_t port);


static const char       *TAG = "main";
static QueueHandle_t     app_sensors_updates;


void
app_main()
{
    esp_err_t      err;
    struct dht     dht;
    struct pms     pms;

    ESP_LOGI(TAG, "Starting up ...");

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

    err = dht_init(&dht, GPIO_NUM_16, DHT_TYPE_AM23xx);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "dht_init failed (%d)", err);
    }

    err = pms_init(&pms, UART_NUM_0, PMS_MODEL_70, PMS_MDPD_03, PMS_CNVMODE_LP);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "pms_init failed (%d)", err);
    }

    ESP_LOGI(TAG, "init ok");

    for (;;) {
        vTaskDelay(TICKS_FROM_MS(30000));
        struct bmp180_trig_conv bmp180_params;
        struct itc_sensor_update *update;
        bmp180_params.oss = BMP180_OSS_1;

        bmp180_trig_conv(&bmp180_params);

        struct dht_trig_conv dht_params;
        dht_params.keep_alive = 0;
        dht_params.updatesq = app_sensors_updates;
        assert(dht_trig_conv(&dht, &dht_params, 0) == pdTRUE);

        struct pms_trig_conv pms_params;
        pms_params.keep_alive = 0;
        pms_params.updatesq = app_sensors_updates;
        assert(pms_trig_conv(&pms, &pms_params, 0) == pdTRUE);

        for (int i = 0; i < 3; i++) {
            xQueueReceive(app_sensors_updates, &update, portMAX_DELAY);

            if (update->status != ESP_OK) {
                ESP_LOGW(TAG, "sensor %d update failed (%d)",
                        update->type, update->status);

                continue;
            }

            switch (update->type) {


            case ITC_SENSOR_TYPE_TAP:

                ESP_LOGI(TAG, "bmp180: temp=%d, pressure=%ld",
                        bmp180_params.update.temp, 
                        bmp180_params.update.pressure);

                break;


            case ITC_SENSOR_TYPE_TAH:

                ESP_LOGI(TAG, "dht: temp=%d, humidity=%d",
                        dht_params.update.temp, dht_params.update.humidity);

                break;


            case ITC_SENSOR_TYPE_PM:

                ESP_LOGI(TAG, "pm: PM1.0 %d, PM2.5 %d, PM10 %d ug/m^3",
                        pms_params.update.pm1d0, pms_params.update.pm2d5,
                        pms_params.update.pm10d);

                break;


            default:

                ESP_LOGW(TAG, "received an unknown update type (%d)",
                        update->type);

                break;


            }
        }

        assert(xQueueReceive(app_sensors_updates, &update, 0) == pdFALSE);
    }
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
