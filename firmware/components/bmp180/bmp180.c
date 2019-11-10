
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_err.h>

#include <driver/i2c.h>

#include "bmp180.h"
#include "ticks.h"


#define BMP180_ADDR            0xEE
#define BMP180_CHECK_ACK       0x01
#define BMP180_LAST_NACK       0x02

#define BMP180_AC1_ADDR        0xAA
#define BMP180_AC2_ADDR        0xAC
#define BMP180_AC3_ADDR        0xAE
#define BMP180_AC4_ADDR        0xB0
#define BMP180_AC5_ADDR        0xB2
#define BMP180_AC6_ADDR        0xB4

#define BMP180_B1_ADDR         0xB6
#define BMP180_B2_ADDR         0xB8

#define BMP180_MB_ADDR         0xBA
#define BMP180_MC_ADDR         0xBC
#define BMP180_MD_ADDR         0xBE

/* Control register address. */

#define BMP180_CR_ADDR         0xF4

/* Control register value to trigger temperature measurement. */

#define BMP180_CR_TEMP         0x2E
#define BMP180_TEMP_CONV_TIME  5          /* ms (actual is 4.5 ms) */

#define BMP180_CR_PRESSURE     0x34
#define BMP180_CR_SCO          0x20       /* Start of conversion bit. */

/* Conversion result register. */

#define BMP180_OUT_ADDR        0xF6
#define BMP180_CHIP_ID_ADDR    0xD0
#define BMP180_CHIP_ID         0x55

#define BMP180_STARTUP_TIME    (10 + 5)   /* ms */
#define BMP180_TIMEOUT         50         /* ms */

#define BMP180_TRIGQ_LENGTH    5


#define BMP180_INIT_CHECK()  ESP_ERROR_CHECK(bmp180_init_err)


static esp_err_t bmp180_read(uint8_t addr, void *data, size_t len);
static uint16_t bmp180_read_u16(uint8_t addr, esp_err_t *err);
static esp_err_t bmp180_write(void *data, size_t len);
static esp_err_t bmp180_fetch_calib_data(void);
static esp_err_t bmp180_conv(long *temp, long *pressure, enum bmp180_oss oss);
static esp_err_t bmp180_read_status(void);
static void bmp180_task(void *params);
static inline void bmp180_fill_update_success(
        struct itc_sensor_update_tap *update, long temp, long pressure);
static inline void bmp180_fill_update_fail(
        struct itc_sensor_update_tap *update);



static const char        *TAG = "bmp180";
static i2c_port_t         bmp180_i2c_port;
static unsigned short     bmp180_calib_ac4, bmp180_calib_ac5, bmp180_calib_ac6;
static short              bmp180_calib_ac1, bmp180_calib_ac2, bmp180_calib_ac3,
                          bmp180_calib_mb, bmp180_calib_mc, bmp180_calib_md,
                          bmp180_calib_b1, bmp180_calib_b2;

/* Indicates whether there was an error during sensors initialization. */

static esp_err_t          bmp180_init_err = ESP_ERR_NO_MEM;
static QueueHandle_t      bmp180_trigq;
static QueueHandle_t      bmp180_updatesq;

/* 
 * Maximum pressure conversion time in ms for different OSS values.
 * Actual values are: 4.5, 7.5, 13.5, 25.5 ms.
 */

static TickType_t         bmp180_oss_conv_ticks[4] = { 
    TICKS_FROM_MS(5), TICKS_FROM_MS(8),
    TICKS_FROM_MS(14), TICKS_FROM_MS(26)
};


/* TODO: add a semaphore for I2C. */


/**
 * Initialize the BMP180 sensor. I2C must be set up already (i2c_driver_install,
 * i2c_param_config)
 *
 * @param port I2C master port.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL There was an I2C communication failure. When requesting
 *                a conversion an error will be reported.
 *     - ESP_ERR_NO_MEM Unrecoverable error. Further operation of the module
 *                      is not possible.
 */

esp_err_t
bmp180_init(i2c_port_t port, QueueHandle_t updates)
{
    esp_err_t         err;
    unsigned char     chip_id;

    bmp180_i2c_port = port;
    bmp180_trigq = xQueueCreate(BMP180_TRIGQ_LENGTH,
            sizeof (struct bmp180_trig_conv *));

    if (bmp180_trigq == NULL) {
        ESP_LOGE(TAG, "failed to create the trigger queue");

        return ESP_ERR_NO_MEM;
    }

    bmp180_updatesq = updates;
    bmp180_init_err = ESP_FAIL;

    /* Sensor initialization over I2C. */

    vTaskDelay(TICKS_FROM_MS(BMP180_STARTUP_TIME));

    err = bmp180_read(BMP180_CHIP_ID_ADDR, &chip_id, sizeof (chip_id));

    if (err != ESP_OK || chip_id != BMP180_CHIP_ID) {
        ESP_LOGW(TAG, "I2C communication verification failed");

        return ESP_FAIL;
    }

    err = bmp180_fetch_calib_data();

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Fetched calibration data: AC1=%d, AC2=%d, AC3=%d, "
                                                "AC4=%u, AC5=%u, AC6=%u, "
                                                "MB=%d, MC=%d, MD=%d, "
                                                "B1=%d, B2=%d",
            bmp180_calib_ac1, bmp180_calib_ac2, bmp180_calib_ac3,
            bmp180_calib_ac4, bmp180_calib_ac5, bmp180_calib_ac6,
            bmp180_calib_mb, bmp180_calib_mc, bmp180_calib_md,
            bmp180_calib_b1, bmp180_calib_b2);
    } else {
        ESP_LOGE(TAG, "bmp180_fetch_calib_data failed (%d)", err);

        return ESP_FAIL;
    }

    /* Start a new task. */

    if (xTaskCreate(&bmp180_task, "bmp180", 1000, NULL, 
                ITC_SENSOR_TASK_PRIO, NULL) != pdPASS) {

        ESP_LOGW(TAG, "xTaskCreate failed");

        /* 
         * The only cause of this may be insufficient memory,
         * but we may still recover from this error: `bmp180_trig_conv()'
         * will just send an update failure directly to the queue.
         */

        return ESP_FAIL;
    }

    bmp180_init_err = ESP_OK;

    return ESP_OK;
}


void
bmp180_trig_conv(struct bmp180_trig_conv *params)
{
    assert(bmp180_init_err != ESP_ERR_NO_MEM);

    if (bmp180_init_err == ESP_FAIL) {
        bmp180_fill_update_fail(&params->update);

        itc_sensor_send(bmp180_updatesq, &params->update);

        return;
    }

    assert(xQueueSendToBack(bmp180_trigq, &params, 0) == pdPASS);
}


/**
 * Read data from BMP180 register. The sequence is:
 *    START -> BMP180_ADDR | WRITE -> ADDR -> STOP ->
 *    START -> BMP180_ADDR | READ  -> READ len BYTES -> STOP.
 *
 * @param addr Register address.
 * @param data Buffer to read data into.
 * @param len  Length of the data to read.
 * @return
 *     - ESP_OK Data read successfully.
 *     - ESP_FAIL I2C failure.
 */

static esp_err_t
bmp180_read(uint8_t addr, void *data, size_t len)
{
    esp_err_t            err;
    i2c_cmd_handle_t     cmd;

    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BMP180_ADDR | I2C_MASTER_WRITE,
            BMP180_CHECK_ACK);

    i2c_master_write_byte(cmd, addr, BMP180_CHECK_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(bmp180_i2c_port, cmd,
            pdMS_TO_TICKS(BMP180_TIMEOUT));

    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to send a read command (%d)", err);

        return ESP_FAIL;
    }

    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BMP180_ADDR | I2C_MASTER_READ,
            BMP180_CHECK_ACK);

    i2c_master_read(cmd, data, len, BMP180_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(bmp180_i2c_port, cmd,
            pdMS_TO_TICKS(BMP180_TIMEOUT));

    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "data read failed (%d)", err); 

        return ESP_FAIL;
    }

    return ESP_OK;
}


/**
 * Read uint16_t from BMP180 register.
 *
 * @param addr Register address.
 * @return Data read from register or 0 if `err' becomes anything but `ESP_OK'.
 */

static uint16_t
bmp180_read_u16(uint8_t addr, esp_err_t *err)
{
    uint8_t      buf[2];
    uint16_t     value;

    value = 0;

    if (*err != ESP_OK) {
        return 0;
    }

    *err = bmp180_read(addr, buf, sizeof (buf));
    value = (buf[0] << 8) | buf[1];

    return value;
}


/**
 * Write data to the sensors.
 *
 * @param data Data buffer.
 * @param len Data length.
 * @return Error codes from i2c_master_cmd_begin().
 */

static esp_err_t
bmp180_write(void *data, size_t len)
{
    esp_err_t            err;
    i2c_cmd_handle_t     cmd;

    /* Read uncompensated temperature. */

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BMP180_ADDR | I2C_MASTER_WRITE,
            BMP180_CHECK_ACK);

    i2c_master_write(cmd, data, len, BMP180_CHECK_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(bmp180_i2c_port, cmd,
            pdMS_TO_TICKS(BMP180_TIMEOUT));

    i2c_cmd_link_delete(cmd);

    return err;
}


/**
 * Read and populate calibration data from BMP180 EEPROM into static variables.
 *
 * @return
 *     - ESP_OK Calibration data read successfully.
 *     - ESP_FAIL I2C failure.
 */

static esp_err_t
bmp180_fetch_calib_data(void)
{
    esp_err_t     err;

    err = ESP_OK;

    bmp180_calib_ac1 = bmp180_read_u16(BMP180_AC1_ADDR, &err);
    bmp180_calib_ac2 = bmp180_read_u16(BMP180_AC2_ADDR, &err);
    bmp180_calib_ac3 = bmp180_read_u16(BMP180_AC3_ADDR, &err);
    bmp180_calib_ac4 = bmp180_read_u16(BMP180_AC4_ADDR, &err);
    bmp180_calib_ac5 = bmp180_read_u16(BMP180_AC5_ADDR, &err);
    bmp180_calib_ac6 = bmp180_read_u16(BMP180_AC6_ADDR, &err);

    bmp180_calib_b1 = bmp180_read_u16(BMP180_B1_ADDR, &err);
    bmp180_calib_b2 = bmp180_read_u16(BMP180_B2_ADDR, &err);

    bmp180_calib_mb = bmp180_read_u16(BMP180_MB_ADDR, &err);
    bmp180_calib_mc = bmp180_read_u16(BMP180_MC_ADDR, &err);
    bmp180_calib_md = bmp180_read_u16(BMP180_MD_ADDR, &err);

    return err;
}


/**
 * Trigger temperature conversion, read uncompensated temperature value
 * from the sensor and calculate the true temperature.
 * If `temp' is not NULL the true temperature is stored within it.
 * If `pressure' is not NULL, then pressure conversion is triggered,
 * an uncompensated value is read, converted into true pressure and stored 
 * within `pressure'.
 *
 * @param temp Temperature destination variable (0.1*C).
 * @param pressure Pressure destination variable (Pa).
 * @return 
 *     - ESP_OK Success.
 *     - ESP_FAIL I2C failure.
 */

static esp_err_t
bmp180_conv(long *temp, long *pressure, enum bmp180_oss oss)
{
    uint8_t           temp_cmd[2] = { BMP180_CR_ADDR, BMP180_CR_TEMP },
                      pressure_cmd[2] = {
                          BMP180_CR_ADDR, 
                          BMP180_CR_PRESSURE + (oss << 6)
                      },
                      out[3];
    long              ut, x1, x2, x3, b3, b5, b6, p;
    unsigned long     up, b4, b7;
    esp_err_t         err;

    assert(temp != NULL || pressure != NULL);

    err = bmp180_read_status();

    if (err == ESP_FAIL) {
        
        /* Unexpected behaviour. */

        ESP_LOGW(TAG, "A conversion is still running");

        return ESP_FAIL;
    }

    /* Trigger temperature conversion. */

    err = bmp180_write(temp_cmd, sizeof (temp_cmd));

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to trigger temp conversion (%d)", err);

        return ESP_FAIL;
    }

    vTaskDelay(TICKS_FROM_MS(BMP180_TEMP_CONV_TIME));

    err = bmp180_read_status();

    if (err == ESP_FAIL) {

        /* Unexpected behaviour. */

        ESP_LOGW(TAG, "A temperature conversion is still running.");

        return ESP_FAIL;
    }

    /* 
     * Read uncompensated temperature and
     * calculate the true temperature (taken from the datasheet).
     */

    ut = bmp180_read_u16(BMP180_OUT_ADDR, &err);
    x1 = (ut - bmp180_calib_ac6) * bmp180_calib_ac5 >> 15;
    x2 = (bmp180_calib_mc << 11) / (x1 + bmp180_calib_md);
    b5 = x1 + x2;
    *temp = (b5 + 8) >> 4;

    if (pressure != NULL) {
        
        /* Trigger pressure conversion. */

        err = bmp180_write(pressure_cmd, sizeof (pressure_cmd));

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to trigger pressure conversion (%d)", err);

            return ESP_FAIL;
        }

        vTaskDelay(bmp180_oss_conv_ticks[oss]);

        err = bmp180_read_status();

        if (err == ESP_FAIL) {

            /* Unexpected behaviour. */

            ESP_LOGW(TAG, "A pressure conversion is still running.");

            return ESP_FAIL;
        }

        /*
         * Read uncompensated pressure and
         * calculate the true pressure (taken from the datasheet).
         */

        err = bmp180_read(BMP180_OUT_ADDR, out, sizeof (out));

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "pressure data read failed (%d)", err);

            return ESP_FAIL;
        }

        up = ((out[0] << 16) | (out[1] << 8) | out[2]) >> (8 - oss);
        b6 = b5 - 4000;
        x1 = bmp180_calib_b2 * (b6 * b6 >> 12) >> 11;
        x2 = bmp180_calib_ac2 * b6 >> 11;
        x3 = x1 + x2;
        b3 = (((bmp180_calib_ac1 * 4 + x3) << oss) + 2) / 4;
        x1 = bmp180_calib_ac3 * b6 >> 13;
        x2 = bmp180_calib_b1 * (b6 * b6 >> 12) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        b4 = bmp180_calib_ac4 * (unsigned long) (x3 + 32768) >> 15;
        b7 = (up - b3) * (50000 >> oss);
        p = b7 < 0x80000000 ? b7 * 2 / b4 : b7 / b4 * 2;
        x1 = (p >> 8) * (p >> 8);
        x1 = x1 * 3038 >> 16;
        x2 = -7357 * p >> 16;
        p = p + ((x1 + x2 + 3791) >> 4);
        *pressure = p;
    }

    return err;
}


/**
 * Check if a conversion is still running.
 *
 * @return
 *     - ESP_OK Sensor is ready for a new conversion.
 *     - ESP_FAIL A conversion is still running.
 */

static esp_err_t
bmp180_read_status(void)
{
    esp_err_t         err;
    unsigned char     cr;

    err = bmp180_read(BMP180_CR_ADDR, &cr, 1);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read failed: %d", err);

        return ESP_FAIL;
    }

    return (cr & BMP180_CR_SCO) == 0 ? ESP_OK : ESP_FAIL;
}


/**
 * BMP180 task used to handle the trigger queue.
 *
 * @param params Parameters passed to `xTaskCreate'. Always `NULL'.
 */

static void
bmp180_task(void *p)
{
    struct bmp180_trig_conv    *params;

    for (;;) {
        xQueueReceive(bmp180_trigq, &params, portMAX_DELAY);

        params->update.upd.status = bmp180_conv(&params->update.temp,
                &params->update.pressure, params->oss);

        itc_sensor_send(bmp180_updatesq, &params->update);
    }
}


static inline void
bmp180_fill_update_success(struct itc_sensor_update_tap *update,
        long temp, long pressure)
{
    update->upd.status = ESP_OK;
    update->upd.type = ITC_SENSOR_TYPE_TAP;
    update->temp = temp;
    update->pressure = pressure;
}


static inline void
bmp180_fill_update_fail(struct itc_sensor_update_tap *update)
{
    update->upd.status = ESP_FAIL;
    update->upd.type = ITC_SENSOR_TYPE_TAP;
}
