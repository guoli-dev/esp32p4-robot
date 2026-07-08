idf_component_register(
    SRCS "sht30.c" "mq135.c" "mq136.c" "env_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_adc display
)
