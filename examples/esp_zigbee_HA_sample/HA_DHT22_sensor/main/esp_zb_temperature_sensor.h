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

#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   4000    /* 4000 millisecond */
#define HA_ESP_SENSOR_ENDPOINT          10      /* esp temperature sensor device endpoint, used for temperature measurement */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

// DHT22 data pin
#define DHT22_GPIO                      (14)
// DHT22 power pin
#define DHT22_POWER_GPIO                (10)
// Deep sleep second
#define WAKE_UP_TIME_SEC                (10)
// Maximum time to force a device report
#define MUST_SYNC_MINIMUM_TIME          UINT16_C(65535) // 1 day in seconds
// time to send the device to deep sleep when Zigbee radio is on
#define TIME_TO_SLEEP_ZIGBEE_ON         UINT32_C(30 * 1000) // milliseconds
// time to send the device to deep sleep when Zigbee radio is off
#define TIME_TO_SLEEP_ZIGBEE_OFF        UINT32_C(500) // milliseconds



#define HUMIDITY_REPORT                 (1 << 0)
#define TEMPERATURE_REPORT              (1 << 1)

// should using zigbee to push data
#define SHALL_ENABLE_REPORT             (1 << 0)


// Temperature
#define ESP_TEMP_SENSOR_MIN_VALUE       (-40)   /* Local sensor min measured value (degree Celsius) */
#define ESP_TEMP_SENSOR_MAX_VALUE       (80)    /* Local sensor max measured value (degree Celsius) */

// Humidity
#define ESP_RELATIVE_HUMIDITY_SENSOR_MIN_VALUE       (0)      /* Relative Humidity Measurement value min */
#define ESP_RELATIVE_HUMIDITY_SENSOR_MAX_VALUE       (100)    /* Relative Humidity Measurement value max */

/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
 */
#define MANUFACTURER_NAME               "\x09""ESPRESSIF"
#define MODEL_IDENTIFIER                "\x07"CONFIG_IDF_TARGET
#define ESP_PRODUCT_URL                 "\x2B""https://github.com/weixiangmeng521"
#define ESP_DATE_CODE                   "\x08""20250916"

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }
