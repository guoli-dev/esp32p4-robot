idf_component_register(
    SRCS "motor_control.c"
         "encoder.c"
         "mpu6050.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer display
)
