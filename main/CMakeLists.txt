idf_component_register(
    SRCS "main_btn.c"
    INCLUDE_DIRS "."
    REQUIRES hardware motion interaction camera vision c5_bridge sr_slave sensors tof_avoid display vision_ai
)
