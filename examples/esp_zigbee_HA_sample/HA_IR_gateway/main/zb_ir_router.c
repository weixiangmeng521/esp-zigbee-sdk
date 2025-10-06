#include <math.h>
#include "zb_ir_router.h"
#include "sensor_driver.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "hal/gpio_types.h"
#include "driver/rmt_tx.h"
#include "ir_nec_encoder.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>
#include "esp_pm.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "zcl/esp_zigbee_zcl_illuminance_meas.h"


// #if !defined CONFIG_ZB_ZCZR
// #error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
// #endif

static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t nec_encoder = NULL;
// generic device type
uint8_t generic_device_type = 0xFF; 


static const char *TAG = "ESP_ZB_IR_ROUTER";

typedef struct zbstring_s {
    uint8_t len;
    char data[];
} ESP_ZB_PACKED_STRUCT
zbstring_t;

/********************* Define functions **************************/
static float zb_s16_to_lux(int16_t value)
{
    return 1.0 * value / 100;
}


/**
 * @brief 
 */
static void update_lux_sensor_data(float lux){
    // Zigbee ZCL 属性更新
    uint16_t illuminance = (uint16_t)round(lux); // 可根据需要做转换
    esp_zb_zcl_status_t state = esp_zb_zcl_set_attribute_val(
        HA_ESP_LUX_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, 
        &illuminance,
        false
    );
    /* Check for error */
    if(state != ESP_ZB_ZCL_STATUS_SUCCESS){
        ESP_LOGE(TAG, "Failed to set illuminance: 0x%x", state);       
        return;
    }

    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_LUX_SENSOR_ENDPOINT;
    ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
    ESP_LOGI(TAG, "Illuminance radio send reported");
}


/**
 * @brief lunimance sensor callbackd
 * 
 * @param lux 
 */
static void lux_sensor_callback(float lux){
    ESP_LOGI(TAG, "Lux: %.2f lx", lux);
    update_lux_sensor_data(lux);
}


static esp_err_t deferred_driver_init(void){
    // initlize sensor
    lux_sensor_driver_init(lux_sensor_callback);

    static bool is_inited = false;
    if (!is_inited) {
        /* 初始化 NEC IR */
        ESP_LOGI(TAG, "create RMT TX channel");
        rmt_tx_channel_config_t tx_channel_cfg = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = IR_RESOLUTION_HZ,
            .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
            .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
            .gpio_num = IR_TX_GPIO_NUM,
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

        ESP_LOGI(TAG, "modulate carrier to TX channel");
        rmt_carrier_config_t carrier_cfg = {
            .duty_cycle = 0.33,
            .frequency_hz = 38000, // 38KHz
        };
        ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

        ESP_LOGI(TAG, "install IR NEC encoder");
        ir_nec_encoder_config_t nec_encoder_cfg = {
            .resolution = IR_RESOLUTION_HZ,
        };
        ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));

        ESP_LOGI(TAG, "enable RMT TX channels");
        ESP_ERROR_CHECK(rmt_enable(tx_channel));

        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}


// BDB启动回调
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}


// Zigbee application signal handler
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
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
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;     
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        /* Device left network or lost connection */
        ESP_LOGW(TAG, "Device left network or lost connection");
        if (!esp_zb_bdb_is_factory_new()) {
            /* If device was previously joined, attempt to rejoin */
            ESP_LOGI(TAG, "Attempting to rejoin previous network...");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);  /* Wait 5 seconds before retry */
        }
        break;        
    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        /* Lost connection to parent router */
        ESP_LOGW(TAG, "Lost connection to parent router");
        if (!esp_zb_bdb_is_factory_new()) {
            /* Schedule reconnection attempt */
            ESP_LOGI(TAG, "Scheduling reconnection attempt...");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 10000);  /* Wait 10 seconds before retry */
        }
        break;        
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        esp_zb_set_node_descriptor_manufacturer_code(ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC);
        break;                           
    default:
        /* ESP_ZB_ZDO_SIGNAL_DEVICE_UNAVAILABLE */
        if (sig_type == 0x3c) {  
            ESP_LOGD(TAG, "ZDO signal: Device Unavailable");
            break;
        }
        /* Reduce log level for common non-critical signals */
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

// 触发IR的发射
void emit_nec_ir_command(bool power)
{
    // 全灯
    // ---NEC frame end: Address=6D82, Command=59A6
    // 关灯
    // ---NEC frame end: Address=6D82, Command=43BC
   
    const ir_nec_scan_code_t scan_code = {
        .address = 0x6D82,
        .command = power ? 0x59A6 : 0x43BC,
    };
    // this example won't send NEC frames in a loop
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // no loop
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_channel, -1));
    esp_zb_lock_release();
    ESP_LOGI(TAG, "NEC transmit done");
}

// Log esp_zb_zcl_cmd_info_t structure
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


// Handle attribute value indication
static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t* attribute)
{
    /* Basic cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
            attribute->data.value) {
            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
            char *string = (char*)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            ESP_LOGI(TAG, "Peer Manufacturer is \"%s\"", string);
            free(string);
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
            attribute->data.value) {
            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
            char *string = (char*)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            ESP_LOGI(TAG, "Peer Model is \"%s\"", string);
            free(string);
        }
    }

    /* Illuminance Measurement cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
            int16_t value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Measured Value is %.2f lux", zb_s16_to_lux(value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MIN_MEASURED_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
            int16_t min_value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Min Measured Value is %.2f lux", zb_s16_to_lux(min_value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MAX_MEASURED_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
            int16_t max_value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Max Measured Value is %.2f lux", zb_s16_to_lux(max_value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_TOLERANCE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
            uint16_t tolerance = attribute->data.value ? *(uint16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Tolerance is %.2f", 1.0 * tolerance / 100);
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_LIGHT_SENSOR_TYPE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
            uint8_t sensor_type = attribute->data.value ? *(uint8_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Device type is %u", sensor_type);
        }        
    }
}


// Handle attribute changes
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(
        TAG, 
        "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", 
        message->info.dst_endpoint, message->info.cluster,
        message->attribute.id, message->attribute.data.size
    );
    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (
                message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && 
                message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
            ) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                // light_driver_set_power(light_state);
                emit_nec_ir_command(light_state);
            }
        }
    }
    return ret;
}


// // Zigbee application task
// static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
// {
//     esp_err_t ret = ESP_OK;
//     switch (callback_id) {
//     case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
//         if (tx_channel != NULL) {
//             ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
//             // emit_IR_state() 内使用 tx_channel
//         }    
//         break;
//     default:
//         ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
//         break;
//     }
//     return ret;
// }

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
        if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
            esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
        }        
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
    esp_app_zb_attribute_handler(message->cluster, &message->attribute);
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


// light on off device
static esp_zb_cluster_list_t *custom_light_device_clusters_create(esp_zb_on_off_light_cfg_t *light_cfg){
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(light_cfg->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(light_cfg->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_cluster_create(&(light_cfg->on_off_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    // /* Add illuminance measurement cluster for attribute reporting */
    // ESP_ERROR_CHECK(esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list, esp_zb_illuminance_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static esp_zb_ep_list_t *custom_on_off_device_ep_create(uint8_t endpoint_id, esp_zb_on_off_light_cfg_t *on_off_cfg, esp_zb_ep_list_t *ep_list){
    esp_zb_endpoint_config_t on_off_ep = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_light_device_clusters_create(on_off_cfg), on_off_ep);
    return ep_list;
}


// light sensor device
static esp_zb_cluster_list_t *custom_illuminance_sensor_clusters_create(esp_zb_light_sensor_cfg_t *light_cfg){
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(light_cfg->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(light_cfg->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    /* Add illuminance measurement cluster for attribute reporting */
    uint16_t min_lux = 0x0001;
    uint16_t max_lux = 0xFFFE;
    uint16_t measured_lux = 0x0000; // initial value
    uint8_t illuminance_sensor_type = 0x01; /* Photodiode */
    esp_zb_attribute_list_t *esp_zb_illuminance_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MIN_MEASURED_VALUE_ID, &min_lux);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MAX_MEASURED_VALUE_ID, &max_lux);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &measured_lux);    
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_LIGHT_SENSOR_TYPE_ID, &illuminance_sensor_type); 
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list, esp_zb_illuminance_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static esp_zb_ep_list_t *custom_illuminance_sensor_ep_create(uint8_t endpoint_id, esp_zb_light_sensor_cfg_t *light_sensor_cfg, esp_zb_ep_list_t *ep_list){
    esp_zb_endpoint_config_t lux_sensor_ep = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_LIGHT_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_illuminance_sensor_clusters_create(light_sensor_cfg), lux_sensor_ep);
    return ep_list;
}





static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create on off endpoint */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_on_off_light_cfg_t on_off_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    custom_on_off_device_ep_create(HA_ESP_LIGHT_ENDPOINT, &on_off_cfg, ep_list);
    /* Create light sensor endpoint */
    esp_zb_light_sensor_cfg_t lux_sensor_cfg = ESP_ZB_DEFAULT_LIGHT_SENSOR_CONFIG();
    custom_illuminance_sensor_ep_create(HA_ESP_LUX_SENSOR_ENDPOINT, &lux_sensor_cfg, ep_list);
    ESP_ERROR_CHECK(esp_zb_device_register(ep_list));

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t reporting_lux_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .ep = HA_ESP_LUX_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 10,
        .u.send_info.delta.u16 = 0,
        .attr_id = ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&reporting_lux_info));    

    // 注册Zigbee应用信号处理函数
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_set_secondary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK));    
    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Starting Zigbee Main Loop");
    esp_zb_stack_main_loop();
}

// Main application entry point
// https://github.com/espressif/esp-idf/blob/v5.5/examples/zigbee/light_sample/HA_on_off_light/main/esp_zb_light.c
void app_main(void)
{
    // Zigbee平台配置
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    // NVS初始化
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
