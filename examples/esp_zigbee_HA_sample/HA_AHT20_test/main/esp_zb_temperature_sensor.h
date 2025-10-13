#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   4000    /* 4000 millisecond */
#define HA_ESP_SENSOR_ENDPOINT          10      /* esp temperature sensor device endpoint, used for temperature measurement */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

// Before sleep second time
#define BEFORE_DEEP_SLEEP_TIME_SEC      (5)
// Maximum time to force a device report
#define MUST_SYNC_MINIMUM_TIME          UINT16_C(65535) // 1 day in seconds
// time to send the device to deep sleep when Zigbee radio is on
#define TIME_TO_SLEEP_ZIGBEE_ON         UINT32_C(30 * 1000) // milliseconds
// time to send the device to deep sleep when Zigbee radio is off
#define TIME_TO_SLEEP_ZIGBEE_OFF        UINT32_C(500) // milliseconds

// upload only when the temperature changes by more than 0.5°C
#define TEMP_DELTA                      0.6f     
// Humidity change exceeds 1%RH before uploading
#define HUM_DELTA                       6.0f     
// should report max time second 1h
#define MAX_SHOULD_REPROT_TIME_SEC      (3600)
// deep sleep second
#define WAKE_UP_TIME_SEC                (30)


#define HUMIDITY_REPORT                 (1 << 0)
#define TEMPERATURE_REPORT              (1 << 1)
#define BATTERY_REPORT                  (1 << 2)

// Should using zigbee to push data
#define SHALL_ENABLE_REPORT             (1 << 0)

// Temperature
#define ESP_TEMP_SENSOR_MIN_VALUE       (-40)   /* Local sensor min measured value (degree Celsius) */
#define ESP_TEMP_SENSOR_MAX_VALUE       (80)    /* Local sensor max measured value (degree Celsius) */

// Humidity
#define ESP_RELATIVE_HUMIDITY_SENSOR_MIN_VALUE       (0)      /* Relative Humidity Measurement value min */
#define ESP_RELATIVE_HUMIDITY_SENSOR_MAX_VALUE       (100)    /* Relative Humidity Measurement value max */

#define I2C_MASTER_POWER_IO     10                  /*!< GPIO number for I2C master power */
#define I2C_MASTER_SCL_IO       4                   /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO       5                   /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM          I2C_NUM_0           /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ      100000              /*!< I2C master clock frequency */
#define I2C_PORT_AUTO           -1
#define P_LOW                   0
#define P_HIGHT                 1


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
