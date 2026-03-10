idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES i2s_mic nvs_flash espressif__esp-zigbee-lib
)
