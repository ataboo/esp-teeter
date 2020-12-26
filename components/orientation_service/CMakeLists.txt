idf_component_register(
    SRCS "orientation_service.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES mpu_6050 teeter_common)