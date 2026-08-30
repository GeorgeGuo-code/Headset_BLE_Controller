idf_component_register(
    SRCS "mouse_mode.c"
    INCLUDE_DIRS "include"
    REQUIRES hid_output freertos ble_console touch_sensor
)
