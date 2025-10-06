#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** BH1750 命令 **/
#define BH1750_CMD_POWER_DOWN        0x00
#define BH1750_CMD_POWER_ON          0x01
#define BH1750_CMD_RESET             0x07

#define BH1750_CMD_CONT_H_RES_MODE   0x10  // 连续高分辨率模式 (1lx)
#define BH1750_CMD_CONT_H_RES_MODE2  0x11  // 连续高分辨率模式2 (0.5lx)
#define BH1750_CMD_CONT_L_RES_MODE   0x13  // 连续低分辨率模式 (4lx)
#define BH1750_CMD_ONE_H_RES_MODE    0x20  // 单次高分辨率模式
#define BH1750_CMD_ONE_H_RES_MODE2   0x21
#define BH1750_CMD_ONE_L_RES_MODE    0x23


/** 初始化配置结构 **/
typedef struct {
    i2c_port_t i2c_port;     // I2C 端口号
    gpio_num_t sda_io;       // SDA GPIO
    gpio_num_t scl_io;       // SCL GPIO
    uint32_t clk_speed_hz;   // 时钟频率
    uint8_t address;         // BH1750 I2C 地址 (0x23 或 0x5C)
} bh1750_config_t;

/**
 * @brief 初始化 I2C 和 BH1750 设备
 */
esp_err_t bh1750_init(const bh1750_config_t *config);

/**
 * @brief 设置测量模式
 */
esp_err_t bh1750_set_mode(uint8_t mode);

/**
 * @brief 从 BH1750 读取原始光照数据
 * @param[out] raw_value 两字节原始数据
 */
esp_err_t bh1750_read_raw(uint16_t *raw_value);

/**
 * @brief 从 BH1750 读取光照值 (lux)
 */
esp_err_t bh1750_read_lux(float *lux_value);

#ifdef __cplusplus
}
#endif
