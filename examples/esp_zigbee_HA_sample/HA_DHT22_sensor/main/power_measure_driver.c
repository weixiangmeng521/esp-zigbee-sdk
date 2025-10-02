#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           5
#define I2C_MASTER_SDA_IO           6
#define I2C_MASTER_PORT             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define MAX17048_ADDR               0x36

static const char *TAG = "MAX17043";

// void app_main(void) {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     i2c_param_config(I2C_MASTER_PORT, &conf);
//     i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);

//     uint8_t data[2];
//     while (1) {
//         // MAX17048 Voltage register at 0x02 (2 bytes)
//         i2c_master_write_read_device(I2C_MASTER_PORT, MAX17048_ADDR, (uint8_t[]){0x02}, 1, data, 2, 1000 / portTICK_PERIOD_MS);
//         uint16_t raw_voltage = (data[0] << 8) | data[1];
//         float voltage = (raw_voltage >> 4) * 1.25f / 1000.0f;  // mV → V

//         // MAX17048 SOC register at 0x04 (2 bytes)
//         i2c_master_write_read_device(I2C_MASTER_PORT, MAX17048_ADDR, (uint8_t[]){0x04}, 1, data, 2, 1000 / portTICK_PERIOD_MS);
//         float soc = data[0] + data[1] / 256.0f;

//         ESP_LOGI(TAG, "Voltage=%.3f V, SOC=%.2f %%", voltage, soc);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }



