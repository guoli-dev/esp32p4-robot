idf_component_register(
    SRCS
        i2s_slave_rx.c
        sr_esp_sr.c
        sr_task.c
    INCLUDE_DIRS
        .
    REQUIRES
        driver
        freertos
        esp_timer
        c5_bridge
        motion
        spi_flash   # for SPIFFS model partition
)
