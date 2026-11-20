idf_component_register(
    SRCS "main.c" "balance_loop.c" "../../../main/ports/imu_port.c"
    INCLUDE_DIRS "." "../../../main" "../../../main/ports"
    REQUIRES odrive_uart bike_wifi esp_timer freertos log nvs_flash esp_driver_i2c esp_driver_gpio
)
