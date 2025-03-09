idf_component_register(SRCS "main_app.cpp"
                    PRIV_REQUIRES spi_flash arduino TinyGPS++ droneID nvs_flash lvgl
                    INCLUDE_DIRS "")
