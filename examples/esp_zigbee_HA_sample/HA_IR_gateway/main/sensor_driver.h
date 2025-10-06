#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/** I2C 默认参数 **/
#define BH1750_I2C_ADDR_DEFAULT      0x23  // 另一个可能地址是 0x5C
#define BH1750_DEFAULT_SDA_IO        8
#define BH1750_DEFAULT_SCL_IO        9
#define BH1750_DEFAULT_FREQ_HZ       100000

/**
 * @brief lux sensor callback
 * 
 */
typedef void (*esp_lux_sensor_callback_t)(float lux);

/**
 * @brief 
 * 
 * @param config 
 * @param cb 
 * @return esp_err_t 
 */
esp_err_t lux_sensor_driver_init(
    esp_lux_sensor_callback_t cb
);

#ifdef __cplusplus
} // extern "C"
#endif
