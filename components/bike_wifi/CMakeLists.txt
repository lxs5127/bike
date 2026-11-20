idf_component_register(
    SRCS "wifi_link.c" "wifi_session.c"
    INCLUDE_DIRS "include" "../../main/ports"
    REQUIRES esp_wifi esp_netif esp_event esp_timer freertos lwip json
)
