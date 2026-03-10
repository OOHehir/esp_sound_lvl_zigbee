idf_component_register(
    SRCS "zigbee_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash espressif__esp-zigbee-lib
)
