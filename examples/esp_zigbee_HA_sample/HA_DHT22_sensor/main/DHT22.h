// #pragma once
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

// char dht22_begin(int gpio);
// char dht22_getData();
// float dht22_getTemperature();
// int dht22_getHumidity();

// #ifdef __cplusplus
// }
// #endif

/* 

	DHT22 temperature sensor driver

*/

#ifndef DHT22_H_  
#define DHT22_H_

#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2

#define DHT_GPIO			25

/**
 * Starts DHT22 sensor task
 */
void DHT22_task_start(void);

// == function prototypes =======================================

void 	setDHTgpio(int gpio);
char 	readDHT();
uint16_t 	getHumidity();
uint16_t 	getTemperature();

#endif