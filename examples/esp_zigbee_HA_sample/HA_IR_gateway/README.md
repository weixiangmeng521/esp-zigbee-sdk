| Supported Targets | ESP32-H2 | ESP32-C6 | ESP32-C5 |
| ----------------- | -------- | -------- | -------- |

### reference
https://github.com/espressif/esp-idf/blob/v5.5/examples/zigbee/light_sample/HA_on_off_light/main/esp_zb_light.c

https://github.com/UltronSysController/ULTRON_ZB_COLOR_TEMP_LIGHT/blob/a711dbc713ecdb51e636ebfbac517bab9417b223/main/esp_zb_light.c

### Go to folder
cd ./examples/esp_zigbee_HA_sample/HA_IR_gateway


### call
source /Users/mengweixiang/esp/esp-idf/export.sh

### build
rm -rf build 
idf.py fullclean
idf.py --preview set-target esp32h2
idf.py build
idf.py -p /dev/tty.usbmodem1442401 flash -b 115200

### 抹除数据
idf.py -p /dev/tty.usbmodem1442401  erase-flash
idf.py -p /dev/tty.usbmodem5A360298561  erase-flash

idf.py menuconfig
idf.py fullclean
idf.py build


### 获取端口
ls /dev/tty.*

### monitor
idf.py -p /dev/tty.usbmodem1442401 monitor -b 115200