idf_component_register(
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_rmt nvs_flash 
    SRCS "esp_zb_light.c" "ir_nec_encoder.c" "zcl_utility.c"
)
