idf_component_register(
    SRCS "esp_zb_temperature_sensor.c" "sensor_driver.c" "DHT22.c"
    INCLUDE_DIRS "." 
    REQUIRES driver nvs_flash esp_timer
)

