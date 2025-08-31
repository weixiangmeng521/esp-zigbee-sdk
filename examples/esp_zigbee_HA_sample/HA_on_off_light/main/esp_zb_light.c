/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_light.h"
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

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t nec_encoder = NULL;

static const char *TAG = "ESP_ZB_ON_OFF_LIGHT";

#define EXAMPLE_IR_RESOLUTION_HZ            1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM              24 // GPIO number for IR TX


/********************* Define functions **************************/
static esp_err_t deferred_driver_init(void){
    static bool is_inited = false;
    if (!is_inited) {
        /* 初始化 NEC IR */
        ESP_LOGI(TAG, "create RMT TX channel");
        rmt_tx_channel_config_t tx_channel_cfg = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
            .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
            .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
            .gpio_num = EXAMPLE_IR_TX_GPIO_NUM,
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
            .resolution = EXAMPLE_IR_RESOLUTION_HZ,
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
    default:
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
    ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_channel, -1));
    ESP_LOGI(TAG, "NEC transmit done");        
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

// Zigbee application task
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        if (tx_channel != NULL) {
            ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
            // emit_IR_state() 内使用 tx_channel
        }    
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    /* Initialize Zigbee stack */    
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);
    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };
    // 注册endpoint
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_on_off_light_ep, HA_ESP_LIGHT_ENDPOINT, &info);

    // 注册设备
    esp_zb_device_register(esp_zb_on_off_light_ep);
    // 注册回调
    esp_zb_core_action_handler_register(zb_action_handler);
    // 启动Zigbee
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
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
    // Zigbee平台配置
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
