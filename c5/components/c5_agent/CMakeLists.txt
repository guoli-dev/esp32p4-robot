idf_component_register(
    SRCS "voice_engine.c" "wake_word.c" "local_tts.c"
    INCLUDE_DIRS "."
    REQUIRES c5_audio c5_comm c5_cloud driver freertos
)
