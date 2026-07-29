idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES gesture_detect ble_console ble_stack hid_output ble_hid
                             MPU6050 inv_mpu esp_driver_gpio nvs_flash console)
