| Supported Targets | ESP32-H2 | ESP32-C6 | ESP32-C5 |
| ----------------- | -------- | -------- | -------- |

### Go to folder
cd ./examples/esp_zigbee_HA_sample/HA_DHT22_sensor


### call
source /Users/mengweixiang/esp/esp-idf/export.sh

### build
idf.py set-target esp32h2
rm -rf build 
idf.py fullclean
idf.py build
idf.py -p /dev/tty.usbmodem1441401 flash -b 115200


idf.py menuconfig
idf.py fullclean
idf.py build


### 获取端口
ls /dev/tty.*

### monitor
idf.py -p /dev/tty.usbmodem1441401 monitor -b 115200




```c
// data[0]=0x02, data[1]=0x4D, data[2]=0x01, data[3]=0x20, data[4]=0x70
uint8_t data[5] = {0x02, 0x4D, 0x01, 0x20, 0x70};
uint8_t res1 = data[0] << 8 | data[1];

0x24d
```
I (8820) DHT22_SAFE: data[0]=0x02, data[1]=0xA8, data[2]=0x01, data[3]=0x20, data[4]=0xCB
I (8820) DHT22_SAFE: 1 => 200, 2 => 2A8
