idf_component_register(
    SRCS "esp_zb_temperature_sensor.c" "sensor_driver.c" "DHT22.c" "power_measure_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer esp-zigbee-lib 
)

