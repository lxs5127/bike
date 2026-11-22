idf_component_register(
    SRCS "main.c" "../../../main/control/uart_balance_app.c"
         "../../../main/control/balance_loop.c" "../../../main/ports/imu_port.c"
    INCLUDE_DIRS "../../../main" "../../../main/control" "../../../main/ports"
    REQUIRES odrive_uart bike_wifi esp_timer freertos log nvs_flash esp_driver_i2c esp_driver_gpio
)
