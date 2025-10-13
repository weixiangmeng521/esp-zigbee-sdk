idf_component_register(
    SRCS "esp_zb_temperature_sensor.c" "bmp280.c" "ahtxx.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer esp-zigbee-lib
)

