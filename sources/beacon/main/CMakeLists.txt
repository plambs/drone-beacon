idf_component_register(SRCS "main_app.cpp" "led.cpp" "gps.cpp" "nmea.cpp" "beacon.cpp"
                    PRIV_REQUIRES spi_flash arduino TinyGPS++ droneID
					REQUIRES nvs_flash esp_wifi esp_event esp_netif driver
                    INCLUDE_DIRS ".")
