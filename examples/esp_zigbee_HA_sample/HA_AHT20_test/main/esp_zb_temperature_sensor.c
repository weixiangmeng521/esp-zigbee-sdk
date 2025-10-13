#include "esp_zb_temperature_sensor.h"
#include "bmp280.h"
#include "ahtxx.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_check.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"



const static char *TAG = "esp_zb_temperature_sensor";



void oneshot(
    i2c_master_bus_handle_t *i2c0_bus_hdl, 
    i2c_master_bus_config_t *i2c0_bus_cfg,
    bmp280_config_t *bmp280_dev_cfg,
    bmp280_handle_t *bmp280_dev_hdl,
    ahtxx_config_t *ahtxx_dev_cfg,
    ahtxx_handle_t *ahtxx_dev_hdl
){
    // set default GPIO
    gpio_reset_pin(I2C_MASTER_POWER_IO);
    gpio_set_direction(I2C_MASTER_POWER_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_MASTER_POWER_IO, P_HIGHT);

    ESP_ERROR_CHECK(i2c_new_master_bus(i2c0_bus_cfg, i2c0_bus_hdl));

    // init device
    bmp280_init(*i2c0_bus_hdl, bmp280_dev_cfg, bmp280_dev_hdl);
    if (bmp280_dev_hdl == NULL) {
        ESP_LOGE(TAG, "bmp280 handle init failed");
        assert(bmp280_dev_hdl);
    }

    // init device
    ahtxx_init(*i2c0_bus_hdl, ahtxx_dev_cfg, ahtxx_dev_hdl);
    if (ahtxx_dev_hdl == NULL) {
        ESP_LOGE(TAG, "ahtxx handle init failed");
        assert(ahtxx_dev_hdl);
    }


    ESP_LOGI(TAG, "######################## BMP280 & AHTXX - START #########################");
    
    // handle sensor
    float temperature, pressure, humidity;
    esp_err_t result;
    result = bmp280_get_measurements(*bmp280_dev_hdl, &temperature, &pressure);
    if(result != ESP_OK) {
        ESP_LOGE(TAG, "bmp280 device read failed (%s)", esp_err_to_name(result));
    } else {
        pressure = pressure / 100;
        ESP_LOGI(TAG, "barometric pressure: %.2f hPa", pressure);
    }

    result = ahtxx_get_measurement(*ahtxx_dev_hdl, &temperature, &humidity);
    if(result != ESP_OK) {
        ESP_LOGE(TAG, "ahtxx device read failed (%s)", esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "air temperature:     %.2f °C", temperature);
        ESP_LOGI(TAG, "relative humidity:   %.2f %c", humidity, '%');
    }

    ESP_LOGI(TAG, "######################## BMP280 & AHTXX - END ###########################");

    // free resources
    ESP_ERROR_CHECK(bmp280_delete(*bmp280_dev_hdl));
    ESP_ERROR_CHECK(ahtxx_delete(*ahtxx_dev_hdl));
    ESP_ERROR_CHECK(i2c_del_master_bus(*i2c0_bus_hdl));
    gpio_set_level(I2C_MASTER_POWER_IO, P_LOW);

}



void i2c0_bmp280_task( void *pvParameters ) {
    i2c_master_bus_handle_t  i2c0_bus_hdl = NULL;
    i2c_master_bus_config_t  i2c0_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    bmp280_config_t bmp280_dev_cfg         = BMP280_CONFIG_DEFAULT;
    bmp280_handle_t bmp280_dev_hdl = NULL;
    ahtxx_config_t ahtxx_dev_cfg           = AHT20_CONFIG_DEFAULT;
    ahtxx_handle_t ahtxx_dev_hdl = NULL;

    while (1)
    {
        oneshot(&i2c0_bus_hdl, &i2c0_bus_cfg, &bmp280_dev_cfg, &bmp280_dev_hdl, &ahtxx_dev_cfg, &ahtxx_dev_hdl);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    xTaskCreate(i2c0_bmp280_task, "i2c0_bmp280_task", 4096, NULL, 5, NULL);
}