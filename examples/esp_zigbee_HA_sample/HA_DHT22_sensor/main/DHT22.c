#include "esp_timer.h" 
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DHT22_SAFE";
static float humidity = 0.0f;
static float temperature = 0.0f;
static int pin = 4;

/*
 * wait_for_level - 等待 pin 达到指定电平，带超时
 * returns: elapsed microseconds if level observed, -1 if timeout
 */
static int64_t wait_for_level(int level, int64_t timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) {
            return -1;
        }
        /* 为了尽量短地等待并减少 CPU 占用，这里用小的 busy-wait */
        esp_rom_delay_us(1);
    }
    return esp_timer_get_time() - start;
}


/*
 * 初始化 DHT22 传感器
 * - gpio: 连接 DHT22 的 GPIO 编号
 * - 返回值：成功返回 1，失败返回 0
 */
void setDHTgpio(int gpio){
	pin = gpio;
	// 配置 GPIO
	gpio_reset_pin(pin);
	gpio_set_direction(pin, GPIO_MODE_INPUT);
	gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY); // DHT22 需要上拉
}


/*
 * 安全版 dht22_getData
 * - 每个等待都有限时（us）
 * - 任何超时立刻返回 0 (failure)
 * - 不关全局中断，不会导致 Task WDT
 */
char readDHT()
{
    uint8_t data[5] = {0};
    const int64_t TIMEOUT_RESPONSE = 200;   // 等待 DHT 响应的超时（us）
    const int64_t TIMEOUT_BIT = 200;        // 等待 bit 前导/高电平的超时（us）
    // ensure pin configured output and pull low for start (>18ms)
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(18)); // 18 ms

    // pull high 30~40us
    gpio_set_level(pin, 1);
    esp_rom_delay_us(40);

    // switch to input
    gpio_set_direction(pin, GPIO_MODE_INPUT);

    // DHT should pull low ~80us as response; wait for that low
    if (wait_for_level(0, TIMEOUT_RESPONSE) < 0) {
        ESP_LOGW(TAG, "DHT: no response (wait for initial LOW)");
        return 0;
    }

    // then DHT pulls high ~80us
    if (wait_for_level(1, TIMEOUT_RESPONSE) < 0) {
        ESP_LOGW(TAG, "DHT: no response (wait for initial HIGH)");
        return 0;
    }

    // then ready to send bits: for each of 40 bits read low->high durations
    for (int byte = 0; byte < 5; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            // wait for low (start of bit) - should be about 50us
            if (wait_for_level(0, TIMEOUT_BIT) < 0) {
                ESP_LOGW(TAG, "DHT: timeout waiting for bit start (LOW)");
                return 0;
            }

            // wait for high (this high length determines 0/1)
            int64_t dur = wait_for_level(1, TIMEOUT_BIT);
            if (dur < 0) {
                ESP_LOGW(TAG, "DHT: timeout waiting for bit high");
                return 0;
            }

            // now measure length of HIGH: typical 26~28us (0) or ~70us (1)
            int64_t start = esp_timer_get_time();
            while (gpio_get_level(pin) == 1) {
                if ((esp_timer_get_time() - start) > TIMEOUT_BIT) {
                    break;
                }
            }
            int64_t high_len = esp_timer_get_time() - start;
            if (high_len > 50) { // threshold between 0 and 1 (tunable)
                data[byte] |= (1 << (7 - bit));
            }
        }
    }

	// checksum
	uint8_t sum = ((uint16_t)data[0] + (uint16_t)data[1] + (uint16_t)data[2] + (uint16_t)data[3]) & 0xFF;
	if (sum != data[4]) {
		ESP_LOGW(TAG, "DHT: checksum fail (got 0x%02X expected 0x%02X)", data[4], sum);
		return 0;
	}

	// // for debug
	// ESP_LOGI(TAG, 
	// 		"data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, data[4]=0x%02X",
	// 		data[0], data[1], data[2], data[3], data[4]);

	// ESP_LOGI(TAG, 
	// 		"----\n 1 => %02X\n, 2 => %02X\n, 3 => %.1f%%\n ----\n",
	// 		(uint16_t)data[0] << 8, 
	// 		((uint16_t)data[0] << 8) | data[1],
	// 		(((uint16_t)data[0] << 8) | data[1]) / 10.0f
	// );

	// Calculate temperature and humidity
	uint16_t rawHum = ((uint16_t)data[0] << 8) | data[1];
	humidity = rawHum;

	uint16_t rawTemp = (((data[2] & 0x7F) << 8) | data[3]);
	temperature = rawTemp;
	if (data[2] & 0x80) {
		temperature = -temperature;
	}
    return 1;
}

uint16_t getHumidity(void) {
    return humidity;
}

uint16_t getTemperature(void) {
    return temperature;
}
