idf_component_register(SRCS "hid_output.c"
                    INCLUDE_DIRS "include"
                    REQUIRES ble_stack
                    PRIV_REQUIRES ble_hid bt freertos)
