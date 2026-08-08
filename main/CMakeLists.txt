idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES gesture_detect ble_console ble_stack hid_output ble_hid
                             cmd_config touch_sensor
                             MPU6050 inv_mpu esp_driver_gpio nvs_flash console esp_driver_touch_sens
                             battery_quantity_detection)

# Debug 构建时启用串口触发功能（通过 sdkconfig 的 LOG_LEVEL 判断）
if(CONFIG_LOG_DEFAULT_LEVEL GREATER 3)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC ENABLE_SERIAL_TRIGGER)
endif()
