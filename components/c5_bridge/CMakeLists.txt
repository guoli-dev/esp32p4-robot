idf_component_register(
    SRCS "c5_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos motion sensors
)
