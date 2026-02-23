idf_component_register(SRCS "web-interface.c"
                    INCLUDE_DIRS "."
                    REQUIRES mavespstm driver nvs_flash esp_wifi esp_netif esp_http_server)