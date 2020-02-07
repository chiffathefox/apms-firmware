
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_err.h>

#include <driver/uart.h>

#include "mdbg.h"
#include "ticks.h"
#include "bytes.h"
#include "itc_sensor.h"

#include "pms.h"


enum pms_cmd {
    PMS_CMD_READ = 0xE2,
    PMS_CMD_CHMOD = 0xE1,
    PMS_CMD_SLEEP = 0xE4
};

enum pms_mode {
    PMS_MODE_PASSIVE = 0,
    PMS_MODE_ACTIVE = 1
};

enum pms_pwr_mode {
    PMS_PWR_MODE_SLEEP = 0,
    PMS_PWR_MODE_AWAKE = 1
};


#define PMS_RSP_PACKET_LEN      32
#define PMS_RSP_PACKET_BUF_LEN  (PMS_RSP_PACKET_LEN * 2)
#define PMS_RSP_BUF_LEN         256
#define PMS_TRIGQ_LEN           5
#define PMS_STARTUP_TICKS       TICKS_FROM_MS(500)


static const char    *TAG = "pms";


static void pms_cmd_send(struct pms *pms, enum pms_cmd cmd,
        unsigned char datah, unsigned char datal);
static inline void pms_cmd_read(struct pms *pms);
static inline void pms_cmd_chmod(struct pms *pms, enum pms_mode mode);
static inline void pms_cmd_chpwr(struct pms *pms, enum pms_pwr_mode pwr_mode);
static inline void pms_fill_update_fail(struct itc_sensor_update_pm *update);
static void pms_task(void *param);
static esp_err_t pms_conv(struct pms *pms, struct itc_sensor_update_pm *update);


/* 
 * TODO: implement continuous conversion mode.
 */


esp_err_t
pms_init(struct pms *pms, uart_port_t port, enum pms_model model,
        enum pms_mdpd mdpd, enum pms_cnvmode cnvmode)
{
    uart_config_t     uart_config;
    BaseType_t        rc;
    TickType_t        startup_ticks;

    assert(model == PMS_MODEL_70);

    pms->port = port;
    pms->model = model;
    pms->mdpd = mdpd;
    pms->cnvmode = cnvmode;
    pms->task = NULL;

    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(port, PMS_RSP_BUF_LEN, 0, 0, NULL));

    startup_ticks = PMS_STARTUP_TICKS;

    switch (cnvmode) {


    case PMS_CNVMODE_FAST:

        startup_ticks += PMS_SPINUP_TICKS;

        break;


    case PMS_CNVMODE_LP:

        break;


    }

    vTaskDelay(startup_ticks);

    if (cnvmode == PMS_CNVMODE_LP) {
        pms_cmd_chpwr(pms, PMS_PWR_MODE_SLEEP);
    }

    pms->trigq = xQueueCreate(PMS_TRIGQ_LEN, sizeof (struct pms_trig_conv *));

    if (pms->trigq == NULL) {
        ESP_LOGW(TAG, "xQeueueCreate failed");

        return ESP_FAIL;
    }

    rc = xTaskCreate(pms_task, "pms", 1300, pms, ITC_SENSOR_TASK_PRIO,
            &pms->task);

    if (rc != pdPASS) {
        pms->task = NULL;

        ESP_LOGW(TAG, "xTaskCreate failed");

        return ESP_FAIL;
    }

    return ESP_OK;
}


BaseType_t
pms_trig_conv(struct pms *pms, struct pms_trig_conv *params, TickType_t timeout)
{
    if (pms->task == NULL) {
        pms_fill_update_fail(&params->update);
        itc_sensor_send(params->updatesq, &params->update);

        return pdTRUE;
    }

    return xQueueSendToBack(pms->trigq, &params, timeout);
}


/**
 * @brief Send a command to the sensor over UART
 *
 * @param pms Device handle.
 * @param cmd Command code.
 * @param datah MSB data byte.
 * @param datal LSB data byte.
 */

static void
pms_cmd_send(struct pms *pms, enum pms_cmd cmd,
        unsigned char datah, unsigned char datal)
{
    int             i;
    char            buf[7];
    uint16_t        checksum;

    buf[0] = 0x42;
    buf[1] = 0x4D;
    buf[2] = cmd;
    buf[3] = datah;
    buf[4] = datal;
    checksum = 0;

    for (i = 0; i < 5; i++) {
        checksum += buf[i];
    }

    buf[5] = checksum >> 8;
    buf[6] = checksum & 0xFF;

    assert(uart_write_bytes(pms->port, buf, sizeof (buf)) == sizeof (buf));
    ESP_ERROR_CHECK(uart_wait_tx_done(pms->port, PMS_MAX_CONV_TICKS));
}


/**
 * @brief Send a read command to the sensor.
 * Triggers a conversion on the device.
 *
 * @param pms Device handle.
 */

static inline void
pms_cmd_read(struct pms *pms)
{
    pms_cmd_send(pms, PMS_CMD_READ, 0, 0);
}


/**
 * @brief Change sensor operation mode.
 * The sensor may work in active (continuous conversion) or passive (upon read
 * command) mode.
 * 
 * @param pms Device handle.
 * @param mode Device operation mode.
 */

static inline void
pms_cmd_chmod(struct pms *pms, enum pms_mode mode)
{
    pms_cmd_send(pms, PMS_CMD_CHMOD, 0, mode);
}


/**
 * @brief Set sensor power mode.
 *
 * @param pms Device handle.
 * @param pwr_mode Power mode.
 *     - PMS_PWR_MODE_SLEEP puts sensor into low-power mode.
 *     - PMS_PWR_MODE_AWAKE puts sensor into normal mode.
 */

static inline void
pms_cmd_chpwr(struct pms *pms, enum pms_pwr_mode pwr_mode)
{
    pms_cmd_send(pms, PMS_CMD_SLEEP, 0, pwr_mode);
}


/**
 * @brief Fill an update structure with data from a failed conversion.
 * 
 * @param update Structure to fill.
 */

static inline void
pms_fill_update_fail(struct itc_sensor_update_pm *update)
{
    update->upd.status = ESP_FAIL;
    update->upd.type = ITC_SENSOR_TYPE_PM;
}


/**
 * @brief PMS gatekeeper task.
 * Controls access to the sensor and handles updates from a trigger queue.
 *
 * @param param Device handle.
 */

static void
pms_task(void *param)
{
    int                             first;
    esp_err_t                       err;
    TickType_t                      last_tick, tick;
    struct pms                     *pms;
    struct pms_trig_conv           *params;
    struct itc_sensor_update_pm     last_update;

    pms = param;
    first = 1;
    last_tick = xTaskGetTickCount();
    last_update.upd.status = ESP_OK;
    last_update.upd.type = ITC_SENSOR_TYPE_PM;

    for (;;) {
        mdbg_info();
        xQueueReceive(pms->trigq, &params, portMAX_DELAY);

        tick = xTaskGetTickCount();

        if (!first && tick - last_tick < params->keep_alive) {
            memcpy(&params->update, &last_update,
                    sizeof (struct itc_sensor_update_pm));
        } else {
            err = pms_conv(pms, &last_update);

            if (err == ESP_OK) {
                first = 0;
                last_tick = tick;

                memcpy(&params->update, &last_update,
                        sizeof (struct itc_sensor_update_pm));
            } else {
                pms_fill_update_fail(&params->update);
            }
        }

        itc_sensor_send(params->updatesq, &params->update);
    }
}


static esp_err_t
pms_conv(struct pms *pms, struct itc_sensor_update_pm *update)
{
    int               n, i, left;
    short             frame_length, checksum, computed_checksum;
    unsigned char     buf[PMS_RSP_PACKET_BUF_LEN], *packet, *end;
    TickType_t        tick;

    if (pms->cnvmode == PMS_CNVMODE_LP) {
        pms_cmd_chpwr(pms, PMS_PWR_MODE_AWAKE);
        vTaskDelay(PMS_SPINUP_TICKS);
    }

    ESP_ERROR_CHECK(uart_flush(pms->port));

    tick = xTaskGetTickCount();
    n = uart_read_bytes(pms->port, buf, PMS_RSP_PACKET_LEN, PMS_MAX_CONV_TICKS);
    ESP_LOGD(TAG, "uart_read_bytes: read %d bytes, took %lu ms", 
            n, (xTaskGetTickCount() - tick) * portTICK_RATE_MS);

    if (pms->cnvmode == PMS_CNVMODE_LP) {
        pms_cmd_chpwr(pms, PMS_PWR_MODE_SLEEP);
    }

    if (n != PMS_RSP_PACKET_LEN) {
        ESP_LOGW(TAG, "uart_read_bytes failed: read %d", n);

        return ESP_FAIL;
    }

    /* Parse the received packet. */

    end = buf + n - 1;

    for (packet = buf; packet < end; packet++) {
        if (packet[0] == 0x42 && packet[1] == 0x4D) {
            break;
        }
    }

    if (end == packet) {
        ESP_LOGW(TAG, "no magic bytes were found");

        return ESP_FAIL;
    }

    end++;
    left = PMS_RSP_PACKET_LEN - (intptr_t) (end - packet);

    if (left > 0) {
        n = uart_read_bytes(pms->port, end, left, PMS_MAX_CONV_TICKS);

        if (n != left) {
            ESP_LOGW(TAG, "uart_read_bytes filed: read %d expected %d", n, left);

            return ESP_FAIL;
        }
    }

    checksum = bytes_be_to_u16(packet + 30);
    computed_checksum = 0;

    for (i = 0; i < PMS_RSP_PACKET_LEN - 2; i++) {
        computed_checksum += packet[i];
    }

    if (checksum != computed_checksum) {
        ESP_LOGW(TAG, "checksum check failed (expected %04X, got %04X)",
                checksum, computed_checksum);

        return ESP_FAIL;
    }

    /*
     * Data1-3 is used for calibration on the factory.
     * Data4-6 contain values that present some interest.
     */

    frame_length = bytes_be_to_u16(packet + 2);

    if (frame_length != PMS_RSP_PACKET_LEN - 4) {
        ESP_LOGW(TAG, "unexpected frame_length (%d)", frame_length);

        return ESP_FAIL;
    }

    update->pm1d0 = bytes_be_to_u16(packet + 10);
    update->pm2d5 = bytes_be_to_u16(packet + 12);
    update->pm10d = bytes_be_to_u16(packet + 14);

    return ESP_OK;
}
