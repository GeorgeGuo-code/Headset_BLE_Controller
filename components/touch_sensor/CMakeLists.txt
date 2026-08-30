idf_component_register(
    SRCS "touch_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES hid_output
    PRIV_REQUIRES esp_driver_touch_sens ble_hid freertos
)
