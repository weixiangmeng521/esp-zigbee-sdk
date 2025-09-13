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
#include "driver/gpio.h"


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

// powser GPIO
static int POWER_GPIO = 10;
static const char *TAG = "ESP_TEMP_SENSOR_DRIVER";
TaskHandle_t xHandle = NULL;
static int P_HIGHT = 0;
static int P_LOW = 1;

/**
 * @brief Tasks for updating the sensor value
 *
 * @param arg      Unused value.
 */
static void temp_sensor_driver_value_update(void *arg)
{   
    float temperature = 0;
    float humidity    = 0;  
    while(1){
        gpio_set_level(POWER_GPIO, P_HIGHT);

        // wait warm up
        int warn_up_time = 1000;
        ESP_LOGI(TAG, "Delay %dms for Warm up sensor...",  warn_up_time);
        vTaskDelay(pdMS_TO_TICKS(warn_up_time));

        // read dht data
        int res = readDHT();

        ESP_LOGI(TAG, "DHT Sensor Reading..." );
        temperature = getTemperature() / 10.0f;
        humidity    = getHumidity() / 10.0f;
        
        // if read success and callback function is set, call it
        if (res == 1 && func_ptr) {
            gpio_set_level(POWER_GPIO, P_LOW);
            func_ptr(temperature, humidity);

            if(xHandle != NULL){
                vTaskDelete(xHandle);
                return;
            }
        }

        // deply 1s
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    return (xTaskCreate(temp_sensor_driver_value_update, "sensor_update", 4096, NULL, 10, &xHandle) == pdTRUE) ? ESP_OK : ESP_FAIL;
}


// Initialize temperature sensor
esp_err_t temp_sensor_driver_init(
    temperature_sensor_config_t *config, 
    int data_gpio,
    int power_gpio,
    esp_temp_sensor_callback_t cb
){   
    // set the DHT used pin
    setDHTgpio(data_gpio);
    POWER_GPIO = power_gpio;

    func_ptr = cb;

    // set default GPIO
    gpio_reset_pin(power_gpio);
    gpio_set_direction(power_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(power_gpio, P_LOW);

    if (ESP_OK != temp_sensor_driver_sensor_init(config)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
