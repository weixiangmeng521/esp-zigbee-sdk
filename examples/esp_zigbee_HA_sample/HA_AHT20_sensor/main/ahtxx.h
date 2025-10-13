/**
 * @file ahtxx.h
 * @defgroup drivers ahtxx
 * @{
 *
 * ESP-IDF driver for ahtxx sensor types
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AHTXX_H__
#define __AHTXX_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

#define I2C_AHTXX_DEV_CLK_SPD   UINT32_C(100000) /*!< ahtxx i2c device scl clock frequency (100KHz) */
#define I2C_AHTXX_DEV_ADDR      UINT8_C(0x38)    /*!< ahtxx i2c device address */


/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings for the aht30 sensor type.
 */
#define AHT30_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AHTXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AHTXX_DEV_CLK_SPD,       \
    .sensor_type     = AHTXX_AHT30 }

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings for the aht25 sensor type.
 */
#define AHT25_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AHTXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AHTXX_DEV_CLK_SPD,       \
    .sensor_type     = AHTXX_AHT25 }

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings for the aht21 sensor type.
 */
#define AHT21_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AHTXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AHTXX_DEV_CLK_SPD,       \
    .sensor_type     = AHTXX_AHT21 }

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings for the aht20 sensor type.
 */
#define AHT20_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AHTXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AHTXX_DEV_CLK_SPD,       \
    .sensor_type     = AHTXX_AHT20 }

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings for the aht10 sensor type.
 */
#define AHT10_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AHTXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AHTXX_DEV_CLK_SPD,       \
    .sensor_type     = AHTXX_AHT10 }



/**
 * public enumerator, union, and structure definitions
 */

/**
 * @brief AHTXX sensor types enumerator definition.
 * 
 * @note AHTXX types vary slightly with respect to setup and initialization according to available documentation.
 * The AHT10 and AHT20 are setup through the initialization command.  The AHT21, AHT25 and AHT30 are setup by resetting
 * 0x1b, 0x1c, and 0x1e initializing registers.
 */
typedef enum ahtxx_sensor_types_e {
    AHTXX_AHT10,    /*!< */
    AHTXX_AHT20,
    AHTXX_AHT21,
    AHTXX_AHT25,
    AHTXX_AHT30
} ahtxx_sensor_types_t;

/**
 * @brief AHTXX configuration structure definition.
 */
typedef struct ahtxx_config_s {
    uint16_t          i2c_address;          /*!< ahtxx i2c device address */
    uint32_t          i2c_clock_speed;      /*!< ahtxx i2c device scl clock speed in hz */
    ahtxx_sensor_types_t sensor_type;   /*!< aht sensor type, see `ahtxx_sensor_types_t` enumerator for support sensor types */
} ahtxx_config_t;

/**
 * @brief AHTXX opaque handle structure definition.
 */
typedef void* ahtxx_handle_t;


/**
 * public function and subroutine declarations
 */

/**
 * @brief Initializes an AHTXX device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] ahtxx_config AHTXX device configuration.
 * @param[out] ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_init(const i2c_master_bus_handle_t master_handle, const ahtxx_config_t *ahtxx_config, ahtxx_handle_t *const ahtxx_handle);

/**
 * @brief Reads temperature and relative humidity from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param temperature Temperature in degree Celsius.
 * @param humidity Relative humidity in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_measurement(ahtxx_handle_t handle, float *const temperature, float *const humidity);

/**
 * @brief Similar to `i2c_aht2x_read_measurement` but it includes dewpoint in the results.
 *
 * @param[in] handle AHTXX device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @param[out] humidity Relative humidity in percentage.
 * @param[out] dewpoint Calculated dew-point temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_measurements(ahtxx_handle_t handle, float *const temperature, float *const humidity, float *const dewpoint);

/**
 * @brief Reads busy status flag from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] busy AHTXX is busy when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_busy_status(ahtxx_handle_t handle, bool *const busy);

/**
 * @brief Reads calibration status flag from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] calibrated AHTXX is calibrated when true.  See `i2c_ahtxx_setup` and datasheet for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_calibration_status(ahtxx_handle_t handle, bool *const calibrated);

/**
 * @brief Reads busy and calibrated status flags from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] busy AHTXX is busy when true.
 * @param[out] calibrated AHTXX is calibrated when true.  See `i2c_ahtxx_setup` and datasheet for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_status(ahtxx_handle_t handle, bool *const busy, bool *const calibrated);

/**
 * @brief Issues soft-reset and initializes AHTXX.  See datasheet for details.
 *
 * @param handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_reset(ahtxx_handle_t handle);

/**
 * @brief Removes an AHTXX device from master bus.
 *
 * @param[in] handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_remove(ahtxx_handle_t handle);

/**
 * @brief Removes an AHTXX device from master bus and frees handle.
 * 
 * @param handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_delete(ahtxx_handle_t handle);

/**
 * @brief Converts AHTXX firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* AHTXX firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* ahtxx_get_fw_version(void);

/**
 * @brief Converts AHTXX firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t AHTXX firmware version number.
 */
int32_t ahtxx_get_fw_version_number(void);



/**
 * public constant definitions
 */

#define AHTXX_COMPONENT_NAME              "esp_ahtxx"
/** Version release date  */
#define AHTXX_FW_VERSION_DATE             "2025-06-07"
/** Major version number (X.x.x) */
#define AHTXX_FW_VERSION_MAJOR            1
/** Minor version number (x.X.x) */
#define AHTXX_FW_VERSION_MINOR            2
/** Patch version number (x.x.X) */
#define AHTXX_FW_VERSION_PATCH            7
/** Semantic version number (X.X.X-X) */
#define AHTXX_FW_SEMANTIC_VERSION         "1.2.7-4"
/** Git version hash */
#define AHTXX_FW_GIT_SHORT_SHA            "f32c566"


/**
 * public macro definitions
 */

/** 
 * Macro to print x parameter as a string i.e. enclose x in double quotes. 
 */
#define STR_QUOTES( x ) #x

/** 
 * Macro to create a string of x parameter with all macros fully expanded. 
 */                 
#define STR( x ) STR_QUOTES( x )

/** 
 * Macro to generate current firmware version numbers (major, minor, patch) into a string that is formatted as X.X.X (e.g. 4.0.0). 
 */
#define AHTXX_FW_VERSION_STR                        \
        STR( AHTXX_FW_VERSION_MAJOR ) "." \
        STR( AHTXX_FW_VERSION_MINOR ) "." \
        STR( AHTXX_FW_VERSION_PATCH )

/** 
 * Macro to convert firmware version parameters (major, minor, patch numbers) into an integer (`int32_t`) 
 * value that can be used for comparison purposes.
 * 
 * As an example, FW_VERSION_INT32 >= FW_VERSION_PARAMS_INT32(4, 0, 0).
 */
#define AHTXX_FW_VERSION_PARAMS_INT32( major, minor, patch )        \
        ((major << 16) | (minor << 8) | (patch))

/**
 * Macro to generate current firmware version numbers (major, minor, patch) as an integer (`int32_t`) value that can 
 * be used for comparison purposes.
 * 
 * As an example, FW_VERSION_INT32 >= FW_VERSION_PARAMS_INT32(4, 0, 0).
 */
#define AHTXX_FW_VERSION_INT32            \
        AHTXX_FW_VERSION_PARAMS_INT32(    \
                AHTXX_FW_VERSION_MAJOR,   \
                AHTXX_FW_VERSION_MINOR,   \
                AHTXX_FW_VERSION_PATCH)


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __AHTXX_H__