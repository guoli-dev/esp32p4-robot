idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES c5_agent c5_audio c5_comm c5_ble c5_cloud
)
