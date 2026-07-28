idf_component_register(SRCS "inv_mpu.c"
                    INCLUDE_DIRS "include"
                    REQUIRES MPU6050 inv_mpu_dmp_motion_driver)
