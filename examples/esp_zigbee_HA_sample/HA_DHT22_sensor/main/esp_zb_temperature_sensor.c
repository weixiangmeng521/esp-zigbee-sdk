/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_temperature_sensor Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_temperature_sensor.h"
#include <math.h>
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "sensor_driver.h"
#include "esp_sleep.h"
#include "driver/uart.h"
#include "sys/time.h"
#include "time.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#endif

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

// upload only when the temperature changes by more than 0.5°C
#define TEMP_DELTA 0.5f     
// Humidity change exceeds 1%RH before uploading
#define HUM_DELTA  1.0f     
// last sleep enter time
static RTC_DATA_ATTR struct timeval s_sleep_enter_time;
// last measured temperature
static RTC_DATA_ATTR float last_temperature = 0;
// last measured humidity
static RTC_DATA_ATTR float last_humidity    = 0;

// one shot timer
static esp_timer_handle_t s_oneshot_timer;


static const char *TAG = "ESP_ZB_TEMP_SENSOR";

// Zigbee 规范里常用的无效值
#define ZB_ZCL_ATTR_INT16S_INVALID 0x8000

static int16_t undefined_value = ZB_ZCL_ATTR_INT16S_INVALID;



static int16_t zb_float_to_s16(float temp)
{
    return (int16_t)roundf(temp * 100);
}


// 创建了一个 one-shot timer 对象
static void s_oneshot_timer_callback(void* arg)
{
    /* Enter deep sleep */
    ESP_LOGI(TAG, "Enter deep sleep");
    gettimeofday(&s_sleep_enter_time, NULL);
    esp_deep_sleep_start();
}


// 开始进入睡眠
static void zb_deep_sleep_start(void)
{
    /* Start the one-shot timer */
    const float before_deep_sleep_time_sec = 10;
    ESP_LOGI(TAG, "Start one-shot timer for %.2fs to enter the deep sleep", before_deep_sleep_time_sec);
    ESP_ERROR_CHECK(esp_timer_start_once(s_oneshot_timer, before_deep_sleep_time_sec * 1000000));
}

// upload temperature data.
static void updata_attribute_for_temperature(float temperature){
    if(fabs(temperature - last_temperature) <= TEMP_DELTA) {
        ESP_LOGI(TAG, "It doesnt need to update temperature.");
        return;
    }
    // upload to zigbee
    int16_t temp_s16 = zb_float_to_s16(temperature);
    esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, 
        &temp_s16,
        false
    );
    /* Check for error */
    if(state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS){
        ESP_LOGE(TAG, "Setting temperature attribute failed!");
        return;
    }
    // update last memeory
	last_temperature = temperature;    
    ESP_LOGI(TAG, "Temperature has updated done.");
}

// upload humidity data.
static void updata_attribute_for_humidity(float humidity){
    if(fabs(humidity - last_humidity) <= HUM_DELTA) {
        ESP_LOGI(TAG, "It doesnt need to update humidity.");
        return;
    }    
    int16_t hum_s16 = zb_float_to_s16(humidity);

    // TODO: DHT22 读失败 / 无效值
    // if (hum_s16 == -32768) {
    // }

    esp_zb_zcl_status_t state_hum = esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        &hum_s16,
        false
    );
    /* Check for error */
    if(state_hum != ESP_ZB_ZCL_STATUS_SUCCESS){
        ESP_LOGE(TAG, "Setting humidity attribute failed!");
        return;
    }
    // update last memeory
    last_humidity = humidity;
    ESP_LOGI(TAG, "Humidity has updated done.");
}

// 获取sensor的数据
static void esp_app_temp_sensor_handler(float temperature, float humidity)
{
    ESP_LOGI(TAG, "Temp: %.1f °C, Hum: %.f%%", temperature, humidity);

    /* Update temperature sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);
    updata_attribute_for_temperature(temperature);
    updata_attribute_for_humidity(humidity);
    esp_zb_lock_release();

    // going to sleep  
    zb_deep_sleep_start();
}

// if read data fail, then callback
static void esp_app_temp_sensor_fallback_handler(){
    // going to sleep directly     
    ESP_LOGI(TAG, "Going to sleep directly.");
    ESP_ERROR_CHECK(esp_timer_start_once(s_oneshot_timer, 1));
}


// Callback to start top level commissioning
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}
 
// Initialize the temperature sensor driver
static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(ESP_TEMP_SENSOR_MIN_VALUE, ESP_TEMP_SENSOR_MAX_VALUE);
    if (!is_inited) {
        ESP_RETURN_ON_ERROR(
            temp_sensor_driver_init(
                &temp_sensor_config, 
                DHT22_GPIO, 
                DHT22_POWER_GPIO, 
                esp_app_temp_sensor_handler,
                esp_app_temp_sensor_fallback_handler
            ),
            TAG, "Failed to initialize temperature sensor");

        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

// Handle Zigbee stack signals
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;    
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            ESP_LOGI(TAG, "Reset device");
            esp_zb_factory_reset();
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        ESP_LOGI(TAG, "Can sleep");   
        break;             
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}


// Create a custom temperature sensor endpoint
static esp_zb_cluster_list_t *custom_temperature_sensor_clusters_create(esp_zb_temperature_sensor_cfg_t *sensor)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(sensor->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(sensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(sensor->temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    
    //  Add humidity measurement cluster
    int16_t min_humidity = zb_float_to_s16(ESP_RELATIVE_HUMIDITY_SENSOR_MIN_VALUE);
    int16_t max_humidity = zb_float_to_s16(ESP_RELATIVE_HUMIDITY_SENSOR_MAX_VALUE);
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &min_humidity);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &max_humidity);

    // Add humidity measurement cluster
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}


// // Create a custom temperature sensor endpoint
// static esp_zb_cluster_list_t *custom_humidity_sensor_clusters_create(esp_zb_temperature_sensor_cfg_t *sensor)
// {
//     esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
//     esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(sensor->basic_cfg));
//     ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
//     ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
//     ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
//     ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(sensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
//     ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
//     return cluster_list;
// }


//  Create a custom temperature sensor endpoint
static esp_zb_ep_list_t *custom_temperature_sensor_ep_create(uint8_t endpoint_id, esp_zb_temperature_sensor_cfg_t *temperature_sensor)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_temperature_sensor_clusters_create(temperature_sensor), endpoint_config);
    return ep_list;
}


// //  Create a custom humidity sensor endpoint
// static esp_zb_ep_list_t *custom_humidity_sensor_ep_create(uint8_t endpoint_id)
// {
//     esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
//     esp_zb_endpoint_config_t endpoint_config = {
//         .endpoint = endpoint_id,
//         .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
//         .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
//         .app_device_version = 0
//     };
//     esp_zb_ep_list_add_ep(ep_list, custom_humidity_sensor_clusters_create(humidity_sensor));
//     return ep_list;
// }


static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_SENSOR_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                switch (message->info.cluster) {
                case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:
                    ESP_LOGI(TAG, "Identify pressed");
                    break;
                default:
                    ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
                }
            }
        }
    }
    return ret;
}

// default messahe handler
static esp_err_t zb_default_resp_handler(esp_zb_zcl_cmd_default_resp_message_t *resp)
{
    ESP_LOGI(TAG, "Default Response received:");
    ESP_LOGI(TAG, "  Status: 0x%x", resp->status_code);
    ESP_LOGI(TAG, "  Source Endpoint: %d", resp->info.src_endpoint);
    ESP_LOGI(TAG, "  Destination Endpoint: %d", resp->info.dst_endpoint);
    ESP_LOGI(TAG, "  Command ID: 0x%x", resp->resp_to_cmd);

    // 根据状态判断命令是否成功
    if (resp->status_code == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Command executed successfully!");
    } else {
        ESP_LOGW(TAG, "Command failed with status: 0x%x", resp->status_code);
    }
    return ESP_OK;
}

// action
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break; 
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        ret = zb_default_resp_handler((esp_zb_zcl_cmd_default_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}



// Zigbee stack main task
static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create customized temperature sensor endpoint */
    esp_zb_temperature_sensor_cfg_t sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    /* Set (Min|Max)MeasuredValure */
    sensor_cfg.temp_meas_cfg.min_value = zb_float_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    sensor_cfg.temp_meas_cfg.max_value = zb_float_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);
    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_temperature_sensor_ep_create(HA_ESP_SENSOR_ENDPOINT, &sensor_cfg);

    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t reporting_temp_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 60,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 100,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_temp_info);

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t reporting_hum_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 60,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 100,
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_hum_info);

    // ------------------------------ Register Device ------------------------------
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}



/**
 * @brief deep sleep
 * 
 */
static void zb_deep_sleep_init(void)
{
    /* Within this function, we print the reason for the wake-up and configure the method of waking up from deep sleep.
    This example provides support for two wake-up sources from deep sleep: RTC timer and GPIO. */

    /* The one-shot timer will start when the device transitions to the CHILD state for the first time.
    After 0.3-second delay, the device will enter deep sleep. */

    const esp_timer_create_args_t s_oneshot_timer_args = {
        .callback = &s_oneshot_timer_callback,
        .name = "one-shot"
    };

    ESP_ERROR_CHECK(esp_timer_create(&s_oneshot_timer_args, &s_oneshot_timer));


    // Print the wake-up reason:
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - s_sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - s_sleep_enter_time.tv_usec) / 1000;
    esp_sleep_wakeup_cause_t wake_up_cause = esp_sleep_get_wakeup_cause();
    switch (wake_up_cause) {
    case ESP_SLEEP_WAKEUP_TIMER: {
        ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_EXT1: {
        ESP_LOGI(TAG, "Wake up from GPIO. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        ESP_LOGI(TAG, "Not a deep sleep reset");
        break;
    }

    /* Set the methods of how to wake up: */
    /* 1. RTC timer waking-up */
    ESP_LOGI(TAG, "Enabling timer wakeup, %ds", WAKE_UP_TIME_SEC);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(WAKE_UP_TIME_SEC * 1000000ULL));
    ESP_LOGI(TAG, "Set wakeup timer done");


    // // https://www.nologo.tech/product/esp32/esp32s3/esp32s3supermini/esp32S3SuperMini.html#%E5%B0%BA%E5%AF%B8%E5%9B%BE
    // // ESP32H2 super mini board
    // const int gpio_wakeup_pin = 0;
    // const uint64_t gpio_wakeup_pin_mask = 1ULL << gpio_wakeup_pin;
    // /* The configuration mode depends on your hardware design.
    // Since the BOOT button is connected to a pull-up resistor, the wake-up mode is configured as LOW. */
    // ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(gpio_wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_LOW));

    // /* Also these two GPIO configurations are also depended on the hardware design.
    // The BOOT button is connected to the pull-up resistor, so enable the pull-up mode and disable the pull-down mode.

    // Notice: if these GPIO configurations do not match the hardware design, the deep sleep module will enable the GPIO hold
    // feature to hold the GPIO voltage when enter the sleep, which will ensure the board be waked up by GPIO. But it will cause
    // 3~4 times power consumption increasing during sleep. */
    // ESP_ERROR_CHECK(gpio_pullup_en(gpio_wakeup_pin));
    // ESP_ERROR_CHECK(gpio_pulldown_dis(gpio_wakeup_pin));    
}

// Application main entry point
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // set deep sleep
    zb_deep_sleep_init();

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
