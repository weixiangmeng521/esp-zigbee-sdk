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
#include "DHT22.h"

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
/* update interval in seconds */
static uint16_t interval = 1;

static const char *TAG = "ESP_TEMP_SENSOR_DRIVER";

/**
 * @brief Tasks for updating the sensor value
 *
 * @param arg      Unused value.
 */
static void temp_sensor_driver_value_update(void *arg)
{
    for (;;) {
	    int res = readDHT();
        
		ESP_LOGI(TAG, "DHT Sensor Reading..." );
        float temperature = getTemperature() / 10.0f;
        float humidity    = getHumidity() / 10.0f;  
                                                
		vTaskDelay(5000 / portTICK_PERIOD_MS);

        // if read success and callback function is set, call it
        if (res == 1 && func_ptr) {
            func_ptr(temperature, humidity);
        }
        vTaskDelay(pdMS_TO_TICKS(interval * 1000));
    }
}

/**
 * @brief init temperature sensor
 *
 * @param config      pointer of temperature sensor config.
 */
static esp_err_t temp_sensor_driver_sensor_init(temperature_sensor_config_t *config)
{
    // 2048
    return (xTaskCreate(temp_sensor_driver_value_update, "sensor_update", 4096, NULL, 10, NULL) == pdTRUE) ? ESP_OK : ESP_FAIL;
}


// Initialize temperature sensor
esp_err_t temp_sensor_driver_init(
    temperature_sensor_config_t *config, 
    uint16_t update_interval,
    int gpio,
    esp_temp_sensor_callback_t cb
){   
    // set the DHT used pin
    setDHTgpio(gpio);

    func_ptr = cb;
    interval = update_interval;

    if (ESP_OK != temp_sensor_driver_sensor_init(config)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
