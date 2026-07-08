idf_component_register(
    SRCS "i2s_driver.c" "tts_player.c" "audio_capture.c" "audio_samples.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos
)
