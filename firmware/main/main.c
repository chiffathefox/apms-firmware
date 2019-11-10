
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <esp_err.h>

#include <driver/i2c.h>

#include "bmp180.h"
#include "itc_sensor.h"


#define APP_I2C_PORT       I2C_NUM_0
#define APP_SENSORS_COUNT  3


static esp_err_t app_i2c_master_init(i2c_port_t port);


static const char       *TAG = "main";
static QueueHandle_t     app_sensors_updates;


void
app_main()
{
    esp_err_t     err;

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

    for (;;) {
        struct bmp180_trig_conv params;
        struct itc_sensor_update *update;
        params.oss = BMP180_OSS_1;

        bmp180_trig_conv(&params);

        xQueueReceive(app_sensors_updates, &update, portMAX_DELAY);

        ESP_LOGI(TAG, "status=%d, type=%d", update->status, update->type);

        if (update->status == ESP_OK) {
            ESP_LOGI(TAG, "temp=%ld, pressure=%ld", params.update.temp,
                    params.update.pressure);
        }
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
    conf.sda_io_num = 4;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = 5;
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
