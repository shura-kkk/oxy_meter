idf_component_register(SRCS "driver_max30102.c"
                            "driver_max30102_fifo.c"
                            "port/driver_max30102_impl_esp.c"
                    PRIV_REQUIRES driver
                    INCLUDE_DIRS "." "port")
