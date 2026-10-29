if(CONFIG_BIKE_WIFI_DIAGNOSTIC)
    set(bike_sources "main.c" "wifi_diagnostic.c")
else()
    set(bike_sources "main.c" "bike_app.c" "control/bike_safety.c"
         "link/bike_protocol.c" "link/bike_link_guard.c" "link/bike_udp_peer.c"
         "ports/pc_link_port.c"
         "ports/imu_port.c" "ports/odrive_port.c")
endif()

idf_component_register(
    SRCS ${bike_sources}
    INCLUDE_DIRS "." "control" "link" "ports"
    REQUIRES esp_timer freertos log esp_hw_support esp_wifi esp_netif esp_event nvs_flash lwip esp_driver_i2c esp_driver_gpio
)
