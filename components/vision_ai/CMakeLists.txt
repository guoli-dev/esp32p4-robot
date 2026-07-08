idf_component_register(
    SRCS "vision_ai.cpp"
    INCLUDE_DIRS "."
    REQUIRES human_face_detect pedestrian_detect
)
