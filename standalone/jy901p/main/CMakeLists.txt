idf_component_register(
    SRCS "jy901p.c"
    INCLUDE_DIRS "."
    REQUIRES bike_wifi esp_driver_i2c esp_driver_gpio freertos log nvs_flash esp_timer
)
