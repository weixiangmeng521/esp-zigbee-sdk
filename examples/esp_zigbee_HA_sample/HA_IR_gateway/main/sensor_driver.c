#include "sensor_driver.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "bh1750.h"

/* call back function pointer */
static esp_lux_sensor_callback_t func_ptr;

static const char *TAG = "ESP_BH1750_DRIVER";
TaskHandle_t xHandle = NULL;

/**
 * @brief Tasks for updating the sensor value
 *
 * @param arg      Unused value.
 */
static void lux_sensor_driver_value_update(void *arg)
{   
    float last_lux = -1.0f;
    while(1){
        // read sensor value every 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));

        float lux;
        if (bh1750_read_lux(&lux) == ESP_OK) {
            if(last_lux != lux){
                func_ptr(lux);
                last_lux = lux;
            }
            // ESP_LOGI(TAG, "Lux: %.2f lx", lux);
        }
    }
}

/**
 * @brief init temperature sensor
 */
static esp_err_t lux_sensor_driver_sensor_init(){
    ESP_LOGI(TAG, "Lux sensor value update task start...");
    // 2048
    return (xTaskCreate(lux_sensor_driver_value_update, "sensor_update", 4096, NULL, 10, &xHandle) == pdTRUE) ? ESP_OK : ESP_FAIL;
}


// Initialize temperature sensor
esp_err_t lux_sensor_driver_init(
    esp_lux_sensor_callback_t cb
){   
    bh1750_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io = BH1750_DEFAULT_SDA_IO,
        .scl_io = BH1750_DEFAULT_SCL_IO,
        .clk_speed_hz = BH1750_DEFAULT_FREQ_HZ,
        .address = BH1750_I2C_ADDR_DEFAULT,
    };
    bh1750_init(&cfg);
    func_ptr = cb;
    if (ESP_OK != lux_sensor_driver_sensor_init()) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
