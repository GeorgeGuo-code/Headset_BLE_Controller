idf_component_register(SRCS "cmd_config.c"
                    INCLUDE_DIRS "include"
                    REQUIRES hid_output gesture_detect
                    PRIV_REQUIRES nvs_flash bt)
