/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee temperature sensor driver example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "sensor_driver.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "bmp280.h"
#include "ahtxx.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include <inttypes.h>

/**
 * @brief:
 * This example code shows how to configure temperature sensor.
 *
 * @note:
 * The callback will be called with updated temperature sensor value every $interval seconds.
 *
 */


/* call back function pointer */
static esp_temp_sensor_callback_t func_ptr;
static esp_temp_sensor_fallback_callback_t fallback_func_ptr;

// powser GPIO
static const char *TAG = "AHT20&BMP280_SENSOR_DRIVER";
TaskHandle_t xHandle = NULL;

static i2c_master_bus_handle_t i2c0_bus_hdl;
static i2c_master_bus_config_t i2c0_bus_cfg;
static bmp280_config_t bmp280_dev_cfg        = BMP280_CONFIG_DEFAULT;
static bmp280_handle_t bmp280_dev_hdl;
static ahtxx_config_t ahtxx_dev_cfg          = AHT20_CONFIG_DEFAULT;
static ahtxx_handle_t ahtxx_dev_hdl;
static float temperature                     = 0.0f;
static float humidity                        = 0.0f;
static float pressure                        = 0.0f;
static int i2c_master_power_gpio             = 10;
static int i2c_master_scl_io                 = 4;
static int i2c_master_sad_io                 = 5;


// close
void remove_i2c(){
    if(bmp280_dev_hdl) ESP_ERROR_CHECK(bmp280_delete(bmp280_dev_hdl));
    if(ahtxx_dev_hdl) ESP_ERROR_CHECK(ahtxx_delete(ahtxx_dev_hdl));
    if(i2c0_bus_hdl) ESP_ERROR_CHECK(i2c_del_master_bus(i2c0_bus_hdl));
    return;
}

/**
 * @brief oneshot for measure
 */
int oneshot(){
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c0_bus_cfg, &i2c0_bus_hdl));

    // init device
    bmp280_init(i2c0_bus_hdl, &bmp280_dev_cfg, &bmp280_dev_hdl);
    if (bmp280_dev_hdl == NULL) {
        ESP_LOGE(TAG, "bmp280 handle init failed");
        // close connection
        remove_i2c();
        return -1;
    }

    // init device
    ahtxx_init(i2c0_bus_hdl, &ahtxx_dev_cfg, &ahtxx_dev_hdl);
    if (ahtxx_dev_hdl == NULL) {
        ESP_LOGE(TAG, "ahtxx handle init failed");
        // close connection
        remove_i2c();
        return -1;        
    }


    ESP_LOGI(TAG, "######################## BMP280 & AHTXX - START #########################");
    
    // handle sensor
    esp_err_t result;
    result = bmp280_get_measurements(bmp280_dev_hdl, &temperature, &pressure);
    if(result != ESP_OK) {
        ESP_LOGE(TAG, "bmp280 device read failed (%s)", esp_err_to_name(result));
    } else {
        pressure = pressure / 100;
        ESP_LOGI(TAG, "barometric pressure: %.2f hPa", pressure);
    }

    result = ahtxx_get_measurement(ahtxx_dev_hdl, &temperature, &humidity);
    if(result != ESP_OK) {
        ESP_LOGE(TAG, "ahtxx device read failed (%s)", esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "air temperature:     %.2f °C", temperature);
        ESP_LOGI(TAG, "relative humidity:   %.2f %c", humidity, '%');
    }

    ESP_LOGI(TAG, "######################## BMP280 & AHTXX - END ###########################");

    // free resources
    remove_i2c();
    return 1;
}


/**
 * @brief Tasks for updating the sensor value
 *
 * @param arg      Unused value.
 */
static void temp_sensor_driver_value_update(void *arg)
{       
    int retry_index           = 0;
    int retry_times_list[]    = {160, 240, 320, 400, 480, 560};
    int retry_count           = sizeof(retry_times_list) / sizeof(retry_times_list[0]);

    // set default GPIO
    gpio_reset_pin(i2c_master_power_gpio);
    gpio_set_direction(i2c_master_power_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(i2c_master_power_gpio, P_HIGHT);

    while(1){
        // measure
        int res = oneshot();

        // if read success and callback function is set
        // call it
        if (res == 1 && func_ptr) {
            func_ptr(temperature, humidity, pressure);
            if(xHandle != NULL){
                vTaskDelete(xHandle);
                gpio_set_level(i2c_master_power_gpio, P_LOW);
                return;
            }
        }

        // if over the reading times, then over
        if(retry_index + 1 >= retry_count){
            ESP_LOGI(TAG, "Fail to get AHT20&BMP280 sensor data");
            fallback_func_ptr();
            vTaskDelete(xHandle);
            if(xHandle != NULL){
                vTaskDelete(xHandle);
                gpio_set_level(i2c_master_power_gpio, P_LOW);
                return;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(retry_times_list[retry_index]));
        retry_index++;
    }
}


/**
 * @brief init temperature sensor
 *
 * @param config      pointer of temperature sensor config.
 */
static esp_err_t temp_sensor_driver_sensor_init()
{
    // 2048
    return (xTaskCreate(temp_sensor_driver_value_update, "sensor_update", 4096, NULL, 10, &xHandle) == pdTRUE) ? ESP_OK : ESP_FAIL;
}


// Initialize temperature sensor
esp_err_t temp_sensor_driver_init(
    int scl_io,
    int sad_io,
    int power_gpio,
    esp_temp_sensor_callback_t cb,
    esp_temp_sensor_fallback_callback_t fallback_cb
){   
    // set the ahtxx&bmp280 used pin
    i2c_master_scl_io = scl_io;
    i2c_master_sad_io = sad_io;    
    i2c_master_power_gpio = power_gpio;

    func_ptr = cb;
    fallback_func_ptr = fallback_cb;

    // i2c configration
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = i2c_master_scl_io,
        .sda_io_num = i2c_master_sad_io,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c0_bus_cfg = cfg;

    if (ESP_OK != temp_sensor_driver_sensor_init()) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
