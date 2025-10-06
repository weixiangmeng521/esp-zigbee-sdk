#include "bh1750.h"
#include "esp_log.h"

static const char *TAG = "BH1750";
static bh1750_config_t g_bh1750_cfg;


/**
 * @brief 初始化 I2C 和 BH1750 设备
 * 
 * @param config 
 * @return esp_err_t 
 */
esp_err_t bh1750_init(const bh1750_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    g_bh1750_cfg = *config;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_io,
        .scl_io_num = config->scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->clk_speed_hz,
    };
    ESP_ERROR_CHECK(i2c_param_config(config->i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(config->i2c_port, conf.mode, 0, 0, 0));

    // 上电 + 复位 + 默认模式
    bh1750_set_mode(BH1750_CMD_POWER_ON);
    vTaskDelay(pdMS_TO_TICKS(10));
    bh1750_set_mode(BH1750_CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
    bh1750_set_mode(BH1750_CMD_CONT_H_RES_MODE);

    ESP_LOGI(TAG, "BH1750 initialized on I2C port %d, addr 0x%02X", config->i2c_port, config->address);
    return ESP_OK;
}


/**
 * @brief 设置测量模式
 * 
 * @param mode 
 * @return esp_err_t 
 */
esp_err_t bh1750_set_mode(uint8_t mode)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_bh1750_cfg.address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mode, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_bh1750_cfg.i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set mode 0x%02X", mode);
    }
    return ret;
}

/**
 * @brief 读取原始光照数据
 * 
 * @param raw_value 
 * @return esp_err_t 
 */
esp_err_t bh1750_read_raw(uint16_t *raw_value)
{
    if (!raw_value) return ESP_ERR_INVALID_ARG;

    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_bh1750_cfg.address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_bh1750_cfg.i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *raw_value = (data[0] << 8) | data[1];
    } else {
        ESP_LOGE(TAG, "Read failed: %d", ret);
    }
    return ret;
}

/**
 * @brief 读取光照值 (lux)
 * 
 * @param lux_value 
 * @return esp_err_t 
 */
esp_err_t bh1750_read_lux(float *lux_value)
{
    uint16_t raw;
    esp_err_t ret = bh1750_read_raw(&raw);
    if (ret == ESP_OK) {
        *lux_value = raw / 1.2f;  // BH1750 默认换算系数
    }
    return ret;
}
