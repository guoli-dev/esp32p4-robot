idf_component_register(
    SRCS "vision_core.c"
         "vision_color.c"
         "vision_line.c"
         "vision_gesture.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer
)
