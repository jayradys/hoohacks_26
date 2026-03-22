idf_component_register(SRCS "bt_app_core.c"
                            "main.c" 
                            "ultrasonic.c"
                    REQUIRES driver
                    PRIV_REQUIRES bt nvs_flash
                    INCLUDE_DIRS ".")
