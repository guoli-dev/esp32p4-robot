idf_component_register(
    SRCS "cam_ov5647.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_cam nvs_flash esp_video
)
