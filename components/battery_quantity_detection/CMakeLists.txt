idf_component_register(SRCS "battery_quantity_detection.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_adc esp_timer esp_driver_gpio driver)
