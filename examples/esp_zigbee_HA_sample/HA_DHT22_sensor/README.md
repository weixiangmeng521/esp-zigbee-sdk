| Supported Targets | ESP32-H2 | ESP32-C6 | ESP32-C5 |
| ----------------- | -------- | -------- | -------- |

### Reference
- https://github.com/MeryaneF/esp-zigbee-sdk/blob/dc28b23f0e3f41d051dc9bda5ef70599eee1a175/examples/esp_zigbee_sleep/deep_sleep/main/esp_zb_sleepy_end_device.c

- https://github.com/lmahmutov/esp32_c6_co2_sensor/blob/main/main/esp_zigbee_co2.c

- https://github.com/IgnacioHR/ZigbeeGasMeter/tree/eebfabbd62c466f94656868b1304fd7068300c53

### Go to folder
cd ./examples/esp_zigbee_HA_sample/HA_DHT22_sensor


### call
source /Users/mengweixiang/esp/esp-idf/export.sh

### build
idf.py set-target esp32h2
rm -rf build 
idf.py fullclean
idf.py build
idf.py -p /dev/tty.usbmodem144201 flash -b 115200



idf.py menuconfig
idf.py fullclean
idf.py build


### 获取端口
ls /dev/tty.*

### monitor
idf.py -p /dev/tty.usbmodem144201 monitor -b 115200


### 抹除数据
idf.py -p  /dev/tty.usbmodem144201  erase-flash


### 通过PMOS管供电，GPIO10控制


### Deep Sleep
20s 后自然唤醒

