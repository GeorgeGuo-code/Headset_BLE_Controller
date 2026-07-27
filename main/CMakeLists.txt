idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES gesture_detect ble_console MPU6050 inv_mpu esp_driver_gpio nvs_flash)
