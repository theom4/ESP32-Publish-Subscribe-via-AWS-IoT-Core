idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi espressif__qrcode coreMQTT coreMQTT-Agent driver esp-idf-bmx280
                    wifi_provisioning)