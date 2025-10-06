idf_component_register(
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp-zigbee-lib esp_driver_rmt nvs_flash driver 
    SRCS "zb_ir_router.c" "ir_nec_encoder.c" "sensor_driver.c" "bh1750.c"
)