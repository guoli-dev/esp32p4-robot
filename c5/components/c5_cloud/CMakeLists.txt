idf_component_register(
    SRCS "cloud_api.c" "wifi_mgr.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_client nvs_flash mbedtls esp-tls tcp_transport esp_wifi esp_netif esp_event freertos
)
