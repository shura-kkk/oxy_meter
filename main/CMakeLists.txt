idf_component_register(SRCS "oxy_meter_main.c"
                            "oxy_meter_display_task.c"
                            "oxy_meter_sensor_task.c"
                    REQUIRES driver esp_lcd max30102 algorithm
                    INCLUDE_DIRS ".")
