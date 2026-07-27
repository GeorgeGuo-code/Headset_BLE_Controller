idf_component_register(
    SRCS "gesture_detect.c"
         "gesture_params.c"
         "calib_session.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos nvs_flash MPU6050 inv_mpu
)