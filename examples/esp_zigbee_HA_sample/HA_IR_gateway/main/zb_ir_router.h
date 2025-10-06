#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */

#define INSTALLCODE_POLICY_ENABLE       false    /* enable the install code policy for security */
#define HA_ESP_LIGHT_ENDPOINT           10       /* Endpoint for light device */
#define HA_ESP_LUX_SENSOR_ENDPOINT      11       /* Endpoint for light sensor device */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */
#define MAX_CHILDREN                    10          /* the max amount of connected devices */

#define IR_RESOLUTION_HZ                1000000 // 1MHz resolution, 1 tick = 1us
#define IR_TX_GPIO_NUM                  24 // GPIO number for IR TX

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME           "\x09""ESPRESSIF"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER            "\x07"CONFIG_IDF_TARGET /* Customized model identifier */
#define ESP_PRODUCT_URL                 "\x2B""https://github.com/weixiangmeng521"
#define ESP_DATE_CODE                   "\x08""20250916"

#define ESP_ZB_ZR_CONFIG()                                                              \
    {                                                                                   \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                                           \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,                               \
        .nwk_cfg.zed_cfg = {                                                            \
            .ed_timeout = ED_AGING_TIMEOUT,                                             \
            .keep_alive = ED_KEEP_ALIVE,                                                \
        },                                                                              \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }
