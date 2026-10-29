idf_component_register(
    SRCS "jy901p.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_driver_gpio freertos log
)
