idf_component_register(
    SRCS "display_lcd.c" "display_touch.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd lvgl
)
