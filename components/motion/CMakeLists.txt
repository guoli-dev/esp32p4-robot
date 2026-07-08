idf_component_register(
    SRCS "pid.c"
         "speed_profile.c"
         "odometry.c"
         "imu_fusion.c"
         "waypoint.c"
         "motion_control.c"
    INCLUDE_DIRS "."
    REQUIRES driver hardware
)
