idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES i2s_mic zigbee_sensor nvs_flash
)
