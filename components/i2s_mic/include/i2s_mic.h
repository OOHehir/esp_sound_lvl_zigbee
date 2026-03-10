#pragma once
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Configuration for the I2S microphone.
 * All GPIO numbers are board-specific — set in main before calling i2s_mic_init().
 */
typedef struct {
    int gpio_bclk;   /*!< Bit clock GPIO */
    int gpio_ws;     /*!< Word select (LR clock) GPIO */
    int gpio_din;    /*!< Data in GPIO */
    uint32_t sample_rate;    /*!< e.g. 16000 */
    uint32_t sample_count;   /*!< Samples per RMS window, e.g. 1024 */
} i2s_mic_config_t;

/**
 * Initialise the I2S peripheral and DMA buffers.
 * Must be called once before i2s_mic_read_level().
 */
esp_err_t i2s_mic_init(const i2s_mic_config_t *cfg);

/**
 * Read one RMS sound level.
 * Blocks until `sample_count` samples have been collected.
 *
 * @param[out] out_level  Normalised RMS level in range [0.0, 1.0]
 * @return ESP_OK on success.
 */
esp_err_t i2s_mic_read_level(float *out_level);

/**
 * Release I2S resources.
 */
esp_err_t i2s_mic_deinit(void);

#ifdef __cplusplus
}
#endif
