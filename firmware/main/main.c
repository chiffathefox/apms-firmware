
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <aws_iot_config.h>
#include <aws_iot_log.h>
#include <aws_iot_version.h>
#include <aws_iot_mqtt_client_interface.h>

#include <driver/i2c.h>

#include <lwip/apps/sntp.h>

#include <cJSON.h>

#include "dht.h"
#include "pms.h"
#include "bmp180.h"
#include "ticks.h"
#include "itc_sensor.h"
#include "mdbg.h"


#define APP_I2C_PORT                  I2C_NUM_0
#define APP_SENSORS_COUNT             3

/* For code sanity checks only.        */

#define APP_QUEUE_WDT_TICKS           TICKS_FROM_MS(300000)
#define APP_MIN_CONV_PERIOD           2000                   /* ms */

#define APP_DEFAULT_CONV_PERIOD       CONFIG_APP_DEFAULT_CONV_PERIOD
#define APP_DEFAULT_PMS_SAFETY_DELAY  CONFIG_APP_DEFAULT_PMS_SAFETY_DELAY
#define APP_DEFAULT_MAX_CONV_DELAY    CONFIG_APP_DEFAULT_MAX_CONV_DELAY
#define APP_DEFAULT_WIFI_SSID         CONFIG_APP_DEFAULT_WIFI_SSID
#define APP_DEFAULT_WIFI_PWD          CONFIG_APP_DEFAULT_WIFI_PWD
#define APP_DEFAULT_PUBLISH_COUNT     CONFIG_APP_PUBLISH_COUNT
#define APP_MQTT_PACKET_TIMEOUT       CONFIG_APP_MQTT_PACKET_TIMEOUT
#define APP_MQTT_COMMAND_TIMEOUT      CONFIG_APP_MQTT_COMMAND_TIMEOUT
#define APP_TLS_HANDSHAKE_TIMEOUT     CONFIG_APP_TLS_HANDSHAKE_TIMEOUT
#define APP_MQTT_CLIENT_ID            CONFIG_APP_MQTT_CLIENT_ID
#define APP_MQTT_KEEP_ALIVE           CONFIG_APP_MQTT_KEEP_ALIVE
#define APP_MQTT_MIN_YIELD_TIMEOUT    CONFIG_APP_MQTT_MIN_YIELD_TIMEOUT
#define APP_MQTT_MAX_YIELD_TIMEOUT    CONFIG_APP_MQTT_MAX_YIELD_TIMEOUT
#define APP_PUBLISH_COUNT             CONFIG_APP_PUBLISH_COUNT
#define APP_SAMPLESQ_LENGTH           CONFIG_APP_SAMPLESQ_LENGTH
#define APP_NTP_SERVERNAME            CONFIG_APP_NTP_SERVERNAME


#if (APP_DEFAULT_CONV_PERIOD < APP_MIN_CONV_PERIOD)


#    error APP_DEFAULT_CONV_PERIOD must be >= then APP_MIN_CONV_PERIOD


#endif


#if (APP_SAMPLESQ_LENGTH < APP_MQTT_COMMAND_TIMEOUT * 2 / APP_MIN_CONV_PERIOD)


#    error APP_SAMPLESQ_LENGTH is too small


#endif


#if (APP_MQTT_MIN_YIELD_TIMEOUT >= APP_MQTT_KEEP_ALIVE)


#    error APP_MQTT_MIN_YIELD_TIMEOUT must be smaller than APP_MQTT_KEEP_ALIVE


#endif


#if (APP_MQTT_MAX_YIELD_TIMEOUT < APP_MQTT_MIN_YIELD_TIMEOUT)


#    error APP_MQTT_MAX_YIELD_TIMEOUT must be greater APP_MQTT_MIN_YIELD_TIMEOUT


#endif


#if (APP_MQTT_MAX_YIELD_TIMEOUT > APP_MQTT_KEEP_ALIVE)


#    error APP_MQTT_MAX_YIELD_TIMEOUT must be smaller than APP_MQTT_KEEP_ALIVE


#endif


#define APP_MQTT_TOPIC                ("sensors/" APP_MQTT_CLIENT_ID)
#define APP_MQTT_TOPIC_LENGTH         (sizeof (APP_MQTT_TOPIC) - 1)


#define APP_WIFI_EG_CONNECTED         BIT0
#define APP_TASK_PRIO                 ITC_SENSOR_TASK_PRIO - 1


struct app_data {
    int          temp;         /* The most accurate available temperature. */
    int          temp_coarse;
    long         pressure;
    int          humidity;
    short        pm1d0;
    short        pm2d5;
    short        pm10d;
    time_t       timestamp;   /* UNIX timestamp in seconds. */
};


struct app_conf {
    TickType_t     pms_safety_delay;
    TickType_t     max_conv_delay;
    TickType_t     conv_period;
    const char    *wifi_ssid;
    const char    *wifi_pwd;
};


#define app_data_log(data, fn)                                                 \
    fn(TAG, "data: temp %d 0.1*C; temp_coarse %d 0.1*C; "                      \
            "pressure %ld Pa; humidity %d 0.1%%RH; "                           \
            "PM1.0 %d ug/m^3; PM2.5 %d ug/m^3; PM10 %d ug/m^3; @ %ld s",       \
            (data)->temp, (data)->temp_coarse, (data)->pressure,               \
            (data)->humidity, (data)->pm1d0, (data)->pm2d5, (data)->pm10d,     \
            (data)->timestamp);


#define APP_JSON_ASSIGN(var, fn, ...)                                          \
    (var) = fn(__VA_ARGS__);                                                   \
                                                                               \
    assert((var) != NULL);


#define APP_JSON_ASSIGN_ATO(var, obj)                                          \
    (var) = cJSON_AddArrayToObject(obj, #var);                                 \
                                                                               \
    assert((var) != NULL);


#define APP_JSON_APPEND_NUM(var)                                               \
    APP_JSON_ASSIGN(num, cJSON_CreateNumber, data.var);                        \
    cJSON_AddItemToArray(var, num);


#define app_wait_for_wifi()                                                    \
    xEventGroupWaitBits(app_wifi_eg, APP_WIFI_EG_CONNECTED, false, true,       \
            portMAX_DELAY);


static esp_err_t app_i2c_master_init(i2c_port_t port);
static inline void app_try(const char *name, esp_err_t err);
static void app_conv_task(void *param);
static inline void app_data_init(struct app_data *data);
static inline int app_data_is_ok(struct app_data *data);
static void app_pop_update(struct app_data *data);
static void app_conv_weather_data(struct app_data *data);
static void app_conf_init(struct app_conf *conf);
static inline void app_nvs_init(void);
static inline void app_wifi_init(void);
static esp_err_t app_event_handler(void *ctx, system_event_t *event);
static void app_wifi_connect(void);
static inline void app_aws_iot_init(void);
static void app_aws_iot_task(void *param);
static void app_aws_iot_disconnect_handler(AWS_IoT_Client *client, void *data);
static char *app_json_from_samples(void);
static inline void app_sntp_init(void);


static const char                  *TAG = "main";
static QueueHandle_t                app_sensors_updates, app_samplesq;
static EventGroupHandle_t           app_wifi_eg;
static struct dht_trig_conv         app_dht_params;
static struct pms_trig_conv         app_pms_params;
static struct bmp180_trig_conv      app_bmp180_params;
static AWS_IoT_Client               app_client;

struct dht                          app_dht;
struct pms                          app_pms;
struct app_conf                     app_conf;

extern const uint8_t         aws_root_ca_pem_start[] 
    asm("_binary_aws_root_ca_pem_start");

extern const uint8_t         certificate_pem_crt_start[]
    asm("_binary_certificate_pem_crt_start");

extern const uint8_t         private_pem_key_start[]
    asm("_binary_private_pem_key_start");


void
app_main()
{
    esp_err_t            err;
    BaseType_t           rc;
    enum pms_cnvmode     cnvmode;

    ESP_LOGI(TAG, "Init stage");

    app_sensors_updates = xQueueCreate(APP_SENSORS_COUNT,
            sizeof (struct itc_sensor_update *));

    assert(app_sensors_updates != NULL);

    app_samplesq = xQueueCreate(APP_SAMPLESQ_LENGTH, sizeof (struct app_data));

    assert(app_samplesq != NULL);

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
    app_nvs_init();
    app_conf_init(&app_conf);

    cnvmode = app_conf.conv_period <= PMS_MAX_SEND_TICKS ?
        PMS_CNVMODE_FAST : PMS_CNVMODE_LP;

    ESP_LOGD(TAG, "conv_period = %lu, selecting cnvmode = %d",
            app_conf.conv_period, cnvmode);

    app_try("pms_init", pms_init(&app_pms, UART_NUM_0, PMS_MODEL_70,
                PMS_MDPD_03, cnvmode));

    /* Initizalize trigger structures. */

    app_bmp180_params.oss = BMP180_OSS_1;

    app_dht_params.keep_alive = 0;
    app_dht_params.updatesq = app_sensors_updates;

    app_pms_params.keep_alive = 0;
    app_pms_params.updatesq = app_sensors_updates;

    app_wifi_init();

    /* Used stack high watermark is 3748 bytes. */

    rc = xTaskCreate(app_aws_iot_task, "app_aws_iot_task", 4000, NULL,
            APP_TASK_PRIO, NULL);

    assert(rc == pdTRUE);

    /* 
     * Let the `app_aws_iot_task' do its job and fetch the 
     * current time in the meanwhile.
     */

    app_sntp_init();

    /* Used stack high watermark is 888 bytes. */

    rc = xTaskCreate(app_conv_task, "app_conv_task", 1110, NULL, APP_TASK_PRIO,
            NULL);

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


/**
 * @brief Print an error message if `err' is not `ESP_OK'.
 *
 * @param name Function name.
 * @param err Function return code.
 */

static inline void
app_try(const char *name, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s() failed (%s)", name, esp_err_to_name(err));
    }
}


/**
 * @brief Task that handles sensor conversions.
 *
 * Triggers sensors conversions with a set `app_conf.conv_period' measurement
 * period and collects the data into a queue for `app_aws_iot_task'
 * to publish.
 *
 * @param param NULL.
 */

static void
app_conv_task(void *param)
{
    int                          rc;
    TickType_t                   delay, last_wakeup, qmaxwait, tick;
    enum pms_cnvmode             cnvmode;
    struct app_data              data, data_lowr;
    struct itc_sensor_update    *update;

    delay = last_wakeup = 0;

    for (;;) {
        mdbg_info();
        ticks_delay_until(&last_wakeup, app_conf.conv_period + delay);

        ESP_LOGD(TAG, "triggerring conversions ...");
        app_conv_weather_data(&data);

        /*
         * If `data.temp' == ITC_SENSOR_INVTEMP,
         * then this fuction returns false.
         */

        /* TODO: take a look at oss */

        /* 
         * TODO: it should be possible for the pms component to change
         *       conversion mode on the fly.
         *       Make a public function to put pms into sleep.
         */

        /* TODO: redundant AP that can be fetched from shadow file. */

        rc = pms_is_safe(&app_pms, data.temp, data.humidity, 0);
        cnvmode = pms_cnvmode(&app_pms);

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

        data.timestamp = time(NULL);

        if (app_data_is_ok(&data)) {
            app_data_log(&data, ESP_LOGI);

            rc = pms_is_safe(&app_pms, data.temp, data.humidity, data.pm10d);
            delay = rc ? 0 : delay + app_conf.pms_safety_delay;
        } else {
            app_data_log(&data, ESP_LOGW);
        }

        if (!rc) {
            switch (cnvmode) {


            case PMS_CNVMODE_FAST:

                /* TODO: put pms to sleep. */

                break;


            case PMS_CNVMODE_LP:

                /* The sensor is already asleep. */

                break;


            }
        }

        assert(xQueueReceive(app_sensors_updates, &update, 0) == pdFALSE);

        if (delay > app_conf.max_conv_delay) {
            delay = app_conf.max_conv_delay;
        }

        qmaxwait = last_wakeup + app_conf.conv_period + delay;
        tick = xTaskGetTickCount();
        qmaxwait = qmaxwait > tick ? qmaxwait - tick : 0;

        if (xQueueSendToBack(app_samplesq, &data, qmaxwait) == pdFALSE) {
            ESP_LOGW(TAG, "samples queue is full, overwriting old samples");

            taskENTER_CRITICAL();

            if (xQueueSendToBack(app_samplesq, &data, 0) == pdFALSE) {
                assert(xQueueReceive(app_samplesq, &data_lowr, 0) == pdTRUE);
                assert(xQueueSendToBack(app_samplesq, &data, 0) == pdTRUE);
            }

            taskEXIT_CRITICAL();
        }
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
    data->timestamp = 0;
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

        abort();

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
app_conf_init(struct app_conf *conf)
{

    /* TODO: load configuration from the shadow file. */

    conf->pms_safety_delay = TICKS_FROM_MS(APP_DEFAULT_PMS_SAFETY_DELAY);
    conf->max_conv_delay = TICKS_FROM_MS(APP_DEFAULT_MAX_CONV_DELAY);
    conf->conv_period = TICKS_FROM_MS(APP_DEFAULT_CONV_PERIOD);
    conf->wifi_ssid = APP_DEFAULT_WIFI_SSID;
    conf->wifi_pwd = APP_DEFAULT_WIFI_PWD;
}


static inline void
app_nvs_init(void)
{
    esp_err_t     err;

    err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());

        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);
}


static inline void
app_wifi_init(void)
{
    wifi_config_t     cfg;

    tcpip_adapter_init();

    app_wifi_eg = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_loop_init(app_event_handler, NULL));

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    memset(&cfg, 0, sizeof (wifi_config_t));
    strncpy((char *) cfg.sta.ssid, app_conf.wifi_ssid, sizeof (cfg.sta.ssid));
    strncpy((char *) cfg.sta.password, app_conf.wifi_pwd,
            sizeof (cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}


static esp_err_t
app_event_handler(void *ctx, system_event_t *event)
{
    system_event_info_t    *info;

    info = &event->event_info;

    switch (event->event_id) {


    case SYSTEM_EVENT_STA_START:

        app_wifi_connect();

        break;


    case SYSTEM_EVENT_STA_GOT_IP:

        ESP_LOGI(TAG, "got ip: %s", 
                ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));

        xEventGroupSetBits(app_wifi_eg, APP_WIFI_EG_CONNECTED);

        break;


    case SYSTEM_EVENT_STA_DISCONNECTED:

        xEventGroupClearBits(app_wifi_eg, APP_WIFI_EG_CONNECTED);
        ESP_LOGW(TAG, "disconnect reason: %d", info->disconnected.reason);

        if (info->disconnected.reason == WIFI_REASON_BASIC_RATE_NOT_SUPPORT) {

            /* Switch to 802.11bgn mode. */

            esp_wifi_set_protocol(ESP_IF_WIFI_STA,
                    WIFI_PROTOCAL_11B | WIFI_PROTOCAL_11G | WIFI_PROTOCAL_11N);
        }

        app_wifi_connect();

        break;


    default:

        ESP_LOGD(TAG, "received system event %d", event->event_id);

        break;


    }

    return ESP_OK;
}


static void
app_wifi_connect(void)
{
    esp_err_t         err;
    wifi_config_t     cfg;
    
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &cfg));

    err = esp_wifi_connect();

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Trying to connect to `%s'", cfg.sta.ssid);
    } else {
        ESP_LOGW(TAG, "esp_wifi_connect failed (%s)", esp_err_to_name(err));
    }
}


static inline void
app_aws_iot_init(void)
{
    IoT_Error_t                    rc;
    IoT_Client_Init_Params         mqtt_init_params;
    IoT_Client_Connect_Params      connect_params;

    mqtt_init_params = iotClientInitParamsDefault;
    mqtt_init_params.enableAutoReconnect = false;
    mqtt_init_params.pHostURL = AWS_IOT_MQTT_HOST;
    mqtt_init_params.port = AWS_IOT_MQTT_PORT;
    mqtt_init_params.pRootCALocation = (const char *) aws_root_ca_pem_start;
    mqtt_init_params.pDeviceCertLocation =
        (const char *) certificate_pem_crt_start;

    mqtt_init_params.pDevicePrivateKeyLocation =
        (const char *) private_pem_key_start;

    mqtt_init_params.mqttPacketTimeout_ms = APP_MQTT_PACKET_TIMEOUT;
    mqtt_init_params.mqttCommandTimeout_ms = APP_MQTT_COMMAND_TIMEOUT;
    mqtt_init_params.tlsHandshakeTimeout_ms = APP_TLS_HANDSHAKE_TIMEOUT;
    mqtt_init_params.isSSLHostnameVerify = true;
    mqtt_init_params.disconnectHandler = app_aws_iot_disconnect_handler;
    mqtt_init_params.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&app_client, &mqtt_init_params);

    assert(rc == SUCCESS);

    connect_params = iotClientConnectParamsDefault;
    connect_params.MQTTVersion = MQTT_3_1_1;
    connect_params.pClientID = APP_MQTT_CLIENT_ID;
    connect_params.clientIDLen = strlen(connect_params.pClientID);
    connect_params.keepAliveIntervalInSec = APP_MQTT_KEEP_ALIVE / 1000;
    connect_params.isCleanSession = true;
    
    /* TODO: add Last Will Testamet to detect disconnected sensors. */

    connect_params.isWillMsgPresent = false;

    app_wait_for_wifi();

    do {
        ESP_LOGD(TAG, "trying to open a MQTT connection ...");

        rc = aws_iot_mqtt_connect(&app_client, &connect_params);

        if (rc != SUCCESS) {
            ESP_LOGE(TAG, "aws_iot_mqtt_connect to %s:%d failed (%d)",
                    mqtt_init_params.pHostURL, mqtt_init_params.port, rc);

            vTaskDelay(TICKS_FROM_MS(AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL));
        }
    } while (rc != SUCCESS);

    ESP_LOGI(TAG, "connected to mqtt @ %s:%d", 
            mqtt_init_params.pHostURL, mqtt_init_params.port);

    rc = aws_iot_mqtt_autoreconnect_set_status(&app_client, true);

    assert(rc == SUCCESS);
}


static void
app_aws_iot_task(void *param)
{
    int                            n_pending;
    uint32_t                       yield_timeout;
    IoT_Error_t                    rc;
    IoT_Publish_Message_Params     msg_params;

    app_aws_iot_init();

    n_pending = 0;

    /* QoS0 guarantees at most once delivery. */

    msg_params.qos = QOS0;
    msg_params.isRetained = 0;
    msg_params.payload = NULL;

    for (;;) {
        mdbg_info();

        n_pending = uxQueueMessagesWaiting(app_samplesq);

        if (n_pending >= APP_PUBLISH_COUNT) {
            yield_timeout = APP_MQTT_MIN_YIELD_TIMEOUT;
        } else {
            yield_timeout = (APP_PUBLISH_COUNT - n_pending) * 
                ticks_to_ms(app_conf.conv_period);

            if (yield_timeout < APP_MQTT_MIN_YIELD_TIMEOUT) {
                yield_timeout = APP_MQTT_MIN_YIELD_TIMEOUT;
            } else if (yield_timeout > APP_MQTT_MAX_YIELD_TIMEOUT) {
                yield_timeout = APP_MQTT_MAX_YIELD_TIMEOUT;
            }
        }

        ESP_LOGD(TAG, "n_pending = %d / %d, selecting yield_timeout = %d ms",
                n_pending, APP_PUBLISH_COUNT, yield_timeout);

        rc = aws_iot_mqtt_yield(&app_client, yield_timeout);

        if (rc == NETWORK_ATTEMPTING_RECONNECT) {

            /*
             * No point to continue the loop, since we won't be able to send any
             * data anyway.
             */

            ESP_LOGD(TAG, "attempting reconnect");

            continue;
        }

        n_pending = uxQueueMessagesWaiting(app_samplesq);

        ESP_LOGD(TAG, "samplesq pending messages cur/pub/max: %d / %d / %d",
                n_pending, APP_PUBLISH_COUNT, APP_SAMPLESQ_LENGTH);

        if (n_pending < APP_PUBLISH_COUNT) {

            /* 
             * Sending data in bulk reduces the number of sent MQTT messages,
             * which in turn reduces the cost of AWS IoT operation.
             */

            continue;
        }

        msg_params.payload = app_json_from_samples();
        msg_params.payloadLen = strlen(msg_params.payload);

        ESP_LOGI(TAG, "publish to %s: payloadLen = %d",
                APP_MQTT_TOPIC, msg_params.payloadLen);

        rc = aws_iot_mqtt_publish(&app_client,
                APP_MQTT_TOPIC, APP_MQTT_TOPIC_LENGTH, &msg_params);

        cJSON_free(msg_params.payload);

        if (rc != SUCCESS) {
            ESP_LOGE(TAG, "aws_iot_mqtt_public failed (%d)", rc);
        } else {
            ESP_LOGD(TAG, "published!");
        }
    }
}


static void
app_aws_iot_disconnect_handler(AWS_IoT_Client *client, void *data)
{
    ESP_LOGW(TAG, "Received MQTT disconnect");

    if (client == NULL) {
        ESP_LOGD(TAG, "%s recived NULL as a client?", __func__);

        return;
    }

    assert(aws_iot_is_autoreconnect_enabled(client));
}


static char *
app_json_from_samples(void)
{
    int                 buflen, n;
    char               *str;
    cJSON              *json, *temp, *temp_coarse, *pressure, *humidity,
                       *pm1d0, *pm2d5, *pm10d, *timestamp, *num;
    struct app_data     data;

    APP_JSON_ASSIGN(json, cJSON_CreateObject);
    APP_JSON_ASSIGN_ATO(temp, json);
    APP_JSON_ASSIGN_ATO(temp_coarse, json);
    APP_JSON_ASSIGN_ATO(pressure, json);
    APP_JSON_ASSIGN_ATO(humidity, json);
    APP_JSON_ASSIGN_ATO(pm1d0, json);
    APP_JSON_ASSIGN_ATO(pm2d5, json);
    APP_JSON_ASSIGN_ATO(pm10d, json);
    APP_JSON_ASSIGN_ATO(timestamp, json);

    for (n = 0; xQueueReceive(app_samplesq, &data, 0) == pdTRUE; n++) {
        APP_JSON_APPEND_NUM(temp);
        APP_JSON_APPEND_NUM(temp_coarse);
        APP_JSON_APPEND_NUM(pressure);
        APP_JSON_APPEND_NUM(humidity);
        APP_JSON_APPEND_NUM(pm1d0);
        APP_JSON_APPEND_NUM(pm2d5);
        APP_JSON_APPEND_NUM(pm10d);
        APP_JSON_APPEND_NUM(timestamp);
    }

    buflen = sizeof ("{ \"temp\": [], \"temp_coarse\": [], \"pressure\": [], "
            "\"humidity\": [], \"pm1d0\": [], \"pm2d5\": [], \"pm10d\": [], "
            "\"timestamp\": [] }") + (
                sizeof ("-32768, ") - 1 +
                sizeof ("-32768, ") - 1 +
                sizeof ("-2147483648, ") - 1 +
                sizeof ("-32768, ") - 1 +
                sizeof ("-32768, ") - 1 +
                sizeof ("-32768, ") - 1 +
                sizeof ("-32768, ") - 1 +
                sizeof ("18446744073709551616, ") - 1
            ) * (n + 1);

    ESP_LOGD(TAG, "%s: preallocating %d bytes for %d samples",
            __func__, buflen, n);

    str = cJSON_PrintBuffered(json, buflen, cJSON_False);

    cJSON_Delete(json);

    return str;
}


static inline void
app_sntp_init(void)
{
    time_t         timestamp;
    TickType_t     tick;

    app_wait_for_wifi();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, APP_NTP_SERVERNAME);
    sntp_init();

    /* Wait for the time to get updated. */

    tick = xTaskGetTickCount();

    do {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        timestamp = time(NULL);
    } while (timestamp < (2019 - 1970) * 86400);

    /* 
     * XXX: because `sizeof (time_t) == 4' 
     *      this code will stop working in 2038.
     */

    tick = xTaskGetTickCount() - tick;

    ESP_LOGI(TAG, "synced in %lu ms, time is %ld",
            ticks_to_ms(tick), timestamp);
}
