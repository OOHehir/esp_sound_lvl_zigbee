#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback invoked when the Zigbee network is joined.
 * Use this to start application tasks that depend on the network.
 */
typedef void (*zigbee_sensor_on_joined_cb_t)(void);

/**
 * Configuration for the Zigbee sensor endpoint.
 */
typedef struct {
    uint8_t endpoint;                       /*!< Zigbee endpoint ID (e.g. 1) */
    zigbee_sensor_on_joined_cb_t on_joined; /*!< Called once when network is joined (may be NULL) */
} zigbee_sensor_config_t;

/**
 * Start the Zigbee stack in a dedicated task.
 * Registers an Analog Input cluster on the configured endpoint.
 * Must be called from app_main after nvs_flash_init().
 */
esp_err_t zigbee_sensor_start(const zigbee_sensor_config_t *cfg);

/**
 * Update the Analog Input present_value attribute.
 * Thread-safe — acquires the Zigbee lock internally.
 *
 * @param level  Sound level value to report (0.0–1.0)
 */
esp_err_t zigbee_sensor_set_level(float level);

#ifdef __cplusplus
}
#endif
