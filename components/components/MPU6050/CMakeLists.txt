idf_component_register(SRCS "MPU6050.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_i2c)
