idf_component_register(
    SRCS "ble_console.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash freertos ble_stack
)
