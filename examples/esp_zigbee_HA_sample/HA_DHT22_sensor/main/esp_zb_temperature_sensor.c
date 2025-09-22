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

// last sleep enter time
static RTC_DATA_ATTR struct timeval s_sleep_enter_time;
// last measured temperature
static RTC_DATA_ATTR float last_temperature = 0;
// last measured humidity
static RTC_DATA_ATTR float last_humidity    = 0;
// last measured time
static RTC_DATA_ATTR struct timeval s_last_reported_time;


// one shot timer
static esp_timer_handle_t s_oneshot_timer;
// generic device type
uint8_t generic_device_type = 0xFF; 
// wait report data handle
EventGroupHandle_t report_event_group_handle = NULL;

// data from sensor
static uint16_t tmp_temperature = 0;
// data from sensor
static uint16_t tmp_humidity    = 0;
static const char *TAG = "ESP_ZB_TEMP_SENSOR";

// define function
static void zigbee_main_task(void *pvParameters); 


/**
 * @brief 
 * TODO: 重置按钮，重置上次网络请求信息
 * 
 */




static uint16_t zb_float_to_s16(float temp)
{
    return (uint16_t)roundf(temp * 100);
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
static void zb_deep_sleep_start(float before_deep_sleep_time_sec)
{
    ESP_LOGI(TAG, "Start one-shot timer for %.2fs to enter the deep sleep", before_deep_sleep_time_sec);
    uint64_t sleep_time_us = (uint64_t)(before_deep_sleep_time_sec * 1000000ULL);
    ESP_ERROR_CHECK(esp_timer_start_once(s_oneshot_timer, sleep_time_us));
}

// upload temperature data.
static bool updata_attribute_for_temperature(uint16_t temperature){
    float temp_f = temperature / 10.0f;

    // if(fabs(temp_f - last_temperature) <= TEMP_DELTA) {
    //     ESP_LOGI(TAG, "It doesnt need to update temperature.");
    //     return false;
    // }

    // upload to zigbee
    uint16_t temp_s16 = temperature * 10;
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
        ESP_LOGE(TAG, "Failed to set temperature: 0x%x", state_tmp);        
        return false;
    }
    // update last memeory
	last_temperature = temp_f;
    ESP_LOGI(TAG, "Temperature has updated done.");
    return true;
}


/**
 * @brief report attributes to radio
 * TODO: depends on baits to report
 */
void zb_radio_send_values(uint8_t mapBits){
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

    // report temperature
    if((mapBits & TEMPERATURE_REPORT) != 0){
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
        ESP_LOGI(TAG, "Temperature reported");
    }
    // report humidity
    if((mapBits & HUMIDITY_REPORT) != 0){
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
        ESP_LOGI(TAG, "Humidity reported");
    }
}



/**
 * @brief upload humidity data.
 * 
 * @param humidity 
 */
static bool updata_attribute_for_humidity(uint16_t humidity){
    float hum_f = humidity / 10.0f;

    // // 判断是否需要更新
    // if (fabs(hum_f - last_humidity) <= HUM_DELTA) {
    //     ESP_LOGI(TAG, "It doesnt need to update humidity.");
    //     return false;
    // }    

    uint16_t hum_s16 = humidity * 10;
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
        ESP_LOGE(TAG, "Failed to set humidity: 0x%x", state_hum);
        return false;
    }
    // update last memeory
    last_humidity = hum_f;
    ESP_LOGI(TAG, "Humidity has updated done.");
    return true;
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
 

/**
 * @brief wait report data task
 * 
 * @param arg 
 */
void report_data_task(void *arg){
    // is reboot just now
    bool is_just_reboot = (esp_reset_reason() == ESP_RST_POWERON);

    while (true){
        ESP_LOGI(TAG, "Waiting for sensor data...");
        xEventGroupWaitBits(
            report_event_group_handle,
            SHALL_ENABLE_REPORT,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(250));

        int8_t map_bits = 0;
        ESP_LOGI(TAG, "Temp: %.1f °C, Hum: %.f%%", (tmp_temperature / 10.0f), (tmp_humidity / 10.0f));
        /* Update temperature sensor measured value */
        esp_zb_lock_acquire(portMAX_DELAY);
        bool res1 = updata_attribute_for_temperature(tmp_temperature);
        if(res1) map_bits |= HUMIDITY_REPORT;
        bool res2 = updata_attribute_for_humidity(tmp_humidity);
        if(res2) map_bits |= TEMPERATURE_REPORT;
        // send values
        zb_radio_send_values(map_bits);
        esp_zb_lock_release();
        // going to sleep
        if(is_just_reboot) {
            ESP_LOGI(TAG, "Device restart just now.");
            zb_deep_sleep_start(13.0f);
        }
        if(!is_just_reboot) {
            zb_deep_sleep_start((float)BEFORE_DEEP_SLEEP_TIME_SEC);
        }
        gettimeofday(&s_last_reported_time, NULL);
        // delete task
        vTaskDelete(NULL);
    }
}


/**
 * @brief Initialize the temperature sensor driver
 * 
 * @return esp_err_t 
 */
static esp_err_t init_report_task(void)
{
    BaseType_t ret = xTaskCreate(report_data_task, "report_data_task", 4096, NULL, tskIDLE_PRIORITY, NULL);
    if (ret == pdPASS) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}


// Handle Zigbee stack signals
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    // esp_zb_zdo_signal_leave_params_t *leave_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", init_report_task() ? "failed" : "successful");
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
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
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
        esp_zb_zdo_signal_leave_params_t *leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params && leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            esp_zb_nvram_erase_at_start(true);                                          // erase previous network information.
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING); // steering a new network.
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        // ESP_LOGI(TAG, "Can sleep");
        break;  
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        esp_zb_set_node_descriptor_manufacturer_code(ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC);
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
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_URL_ID, ESP_PRODUCT_URL));   
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_GENERIC_DEVICE_TYPE_ID, &generic_device_type));    
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, ESP_DATE_CODE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(sensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(sensor->temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    //  Add humidity measurement cluster
    uint16_t min_humidity = zb_float_to_s16(ESP_RELATIVE_HUMIDITY_SENSOR_MIN_VALUE);
    uint16_t max_humidity = zb_float_to_s16(ESP_RELATIVE_HUMIDITY_SENSOR_MAX_VALUE);
    uint16_t last_s16_hum = last_humidity == 0 ? ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT : (uint16_t)(last_humidity * 100);
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &last_s16_hum);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &min_humidity);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &max_humidity);

    // Add humidity measurement cluster
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}


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


/**
 * @brief zb_attribute_handler
 * 
 * @param message 
 * @return esp_err_t 
 */
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


void zb_log_esp_zb_zcl_cmd_info_t(const char* prefix, const esp_zb_zcl_cmd_info_t *info)
{
    ESP_LOGI(TAG, "%s:\n  From address(0x%x) src endpoint(%d)\n  To address (0x%x) endpoint(%d)\n    cluster(0x%x) profile(0x%x) transaction(%d)\n    cmd id(0x%x) direction(0x%x) is_common(0x%x)",
        prefix,
        info->src_address.u.short_addr, 
        info->src_endpoint,
        info->dst_address, 
        info->dst_endpoint, 
        info->cluster,
        info->profile,
        info->header.tsn,
        info->command.id,
        info->command.direction,
        info->command.is_common
    );
}

// response to the ESP_ZB_CORE_REPORT_ATTR_CB_ID callback
esp_err_t zb_cmd_default_resp_handler(const esp_zb_zcl_cmd_default_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    zb_log_esp_zb_zcl_cmd_info_t("Command default response", &message->info);
    ESP_LOGI(TAG, "Default Response: Status: 0x%04x, To Command: 0x%x", message->status_code, message->resp_to_cmd);
    return ESP_OK;
}


// response to the ESP_ZB_CORE_REPORT_ATTR_CB_ID callback
esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
        message->status);
    ESP_LOGI(TAG, "Report received from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x) attribute(0x%x) type(0x%x)",
        message->src_address.u.short_addr, 
        message->src_endpoint,
        message->dst_endpoint, 
        message->cluster,
        message->attribute.id,
        message->attribute.data.type
    );
    return ESP_OK;
}


// response to the ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID callback: TODO check if needed
esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x) transaction(0x%x)",
             message->info.src_address.u.short_addr, 
             message->info.src_endpoint,
             message->info.dst_endpoint, 
             message->info.cluster,
             message->info.header.tsn
    );

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response variable: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
            variable->status, 
            message->info.cluster,
            variable->attribute.id, 
            variable->attribute.data.type,
            variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0
        );
        variable = variable->next;
    }

    return ESP_OK;
}

// response to the ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID callback: TODO check if needed
esp_err_t zb_write_attr_resp_handler(const esp_zb_zcl_cmd_write_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Write attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x) transaction(%d)",
        message->info.src_address.u.short_addr, 
        message->info.src_endpoint,
        message->info.dst_endpoint, 
        message->info.cluster,
        message->info.header.tsn
    );

    esp_zb_zcl_write_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Write attribute response: status(%d), attribute(0x%x)",
                    variable->status,
                    variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}


// response to report config command
esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    zb_log_esp_zb_zcl_cmd_info_t("Configure report response", &message->info);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)",
            variable->status, 
            message->info.cluster, 
            variable->direction, 
            variable->attribute_id
        );
        variable = variable->next;
    }

    return ESP_OK;
}

// action
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break; 
    case ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID:
        ret = zb_write_attr_resp_handler((esp_zb_zcl_cmd_write_attr_resp_message_t *)message);
        break;        
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;        
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;        
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        ret = zb_cmd_default_resp_handler((esp_zb_zcl_cmd_default_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}


/**
 * @brief zigbee main task
 * 
 * @param pvParameters 
 */
static void zigbee_main_task(void *pvParameters){
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, // End device
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ED_AGING_TIMEOUT, // 64 minutes
            .keep_alive = ED_KEEP_ALIVE,    // 3 seconds
        },
    };
    esp_zb_sleep_enable(true);
    esp_zb_init(&zb_nwk_cfg);

    /* Create customized temperature sensor endpoint */
    esp_zb_temperature_sensor_cfg_t sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    uint16_t last_s16_tmp = last_temperature == 0 ? ESP_ZB_ZCL_TEMP_MEASUREMENT_MEASURED_VALUE_DEFAULT : (uint16_t)(last_temperature * 100);

    /* Set (Min|Max|Real)MeasuredValure */
    sensor_cfg.temp_meas_cfg.measured_value = last_s16_tmp;
    sensor_cfg.temp_meas_cfg.min_value = zb_float_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    sensor_cfg.temp_meas_cfg.max_value = zb_float_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);
    sensor_cfg.basic_cfg.power_source = 0x03;
    sensor_cfg.basic_cfg.zcl_version = 0x0008;
    
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
        .u.send_info.min_interval = TIME_TO_SLEEP_ZIGBEE_ON / 2000,
        .u.send_info.max_interval = (uint16_t)MUST_SYNC_MINIMUM_TIME,
        // .u.send_info.def_min_interval = 0,
        // .u.send_info.def_max_interval = 1,
        .u.send_info.delta.u16 = 0,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&reporting_temp_info));

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t reporting_hum_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = TIME_TO_SLEEP_ZIGBEE_ON / 2000,
        .u.send_info.max_interval = (uint16_t)MUST_SYNC_MINIMUM_TIME,
        // .u.send_info.def_min_interval = 0,
        // .u.send_info.def_max_interval = 1,
        .u.send_info.delta.u16 = 0,
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&reporting_hum_info));

    // ---------------- Register Device ----------------
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}


// 获取sensor的数据
static void esp_app_temp_sensor_handler(uint16_t temperature, uint16_t humidity)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    ESP_LOGI(TAG, "Current time: %ld s, %ld us", now.tv_sec, now.tv_usec);    
    // if wake up from reset
    bool is_just_reboot = (esp_reset_reason() == ESP_RST_POWERON);
    if(is_just_reboot){
        // going to start zigbee        
        xTaskCreate(zigbee_main_task, "zigbee_main_task", 4096, NULL, tskIDLE_PRIORITY, NULL);
        xEventGroupSetBits(report_event_group_handle, SHALL_ENABLE_REPORT);
        return;
    }

    // diff
    time_t diff = now.tv_sec - s_last_reported_time.tv_sec;
    // current temperature and humidit
    tmp_temperature = temperature;
    tmp_humidity = humidity;

    // check should for update
    if (
        fabsf((temperature  / 10.0f) - last_temperature) >= TEMP_DELTA ||
        fabsf((humidity / 10.0f) - last_humidity) >= HUM_DELTA || 
        diff >= (int)MAX_SHOULD_REPROT_TIME_SEC
    ) {
        // going to start zigbee        
        xTaskCreate(zigbee_main_task, "zigbee_main_task", 4096, NULL, tskIDLE_PRIORITY, NULL);
        xEventGroupSetBits(report_event_group_handle, SHALL_ENABLE_REPORT);
        return;
    }

    // going to sleep
    ESP_LOGI(TAG, "Dont need to update. going to sleep...");
    ESP_ERROR_CHECK(esp_timer_start_once(s_oneshot_timer, 1));
}


/**
 * @brief init dht22 sensor
 * 
 */
static void init_dht22_sensor(void)
{   
    temperature_sensor_config_t temp_sensor_config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(ESP_TEMP_SENSOR_MIN_VALUE, ESP_TEMP_SENSOR_MAX_VALUE);

    esp_err_t ret = temp_sensor_driver_init(
        &temp_sensor_config,
        DHT22_GPIO,
        DHT22_POWER_GPIO,
        esp_app_temp_sensor_handler,
        esp_app_temp_sensor_fallback_handler
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize temperature sensor: %s", esp_err_to_name(ret));
        return;
    }
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


static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    // TODO: explore why this causes a problem
    // When the device enters deep sleep after a 3s period
    // caused by the counter going up one tick
    // the device wakes up on RTC automatically and this
    // is not desired as it will turn on Zigbee radio
    // as it is defined in code

    // .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}


// Application main entry point
void app_main(void)
{   
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    // create event group
    report_event_group_handle = xEventGroupCreate();

    // set deep sleep
    zb_deep_sleep_init();

    /* Start Zigbee stack task */
    init_dht22_sensor();
}
