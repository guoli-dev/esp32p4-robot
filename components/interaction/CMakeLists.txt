idf_component_register(
    SRCS "oled_display.c"
         "button.c"
    INCLUDE_DIRS "."
    REQUIRES driver display
)
