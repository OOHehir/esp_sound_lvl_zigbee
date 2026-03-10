#include "i2s_mic.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "i2s_mic";

/* SPH0645 outputs 24-bit data left-justified in a 32-bit slot.
   Valid audio is in bits [31:8]; shift right 8 to get a signed 24-bit value,
   then extend to 32-bit for arithmetic. */
#define SPH0645_SHIFT 8

static i2s_chan_handle_t s_rx_chan = NULL;
static i2s_mic_config_t  s_cfg    = {0};
static int32_t          *s_buf    = NULL;

esp_err_t i2s_mic_init(const i2s_mic_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");

    s_cfg = *cfg;
    s_buf = malloc(cfg->sample_count * sizeof(int32_t));
    ESP_RETURN_ON_FALSE(s_buf, ESP_ERR_NO_MEM, TAG, "buffer alloc failed");

    /* Channel config */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan), TAG, "new channel failed");

    /* Standard I2S config for SPH0645: 32-bit slot, 16kHz */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(cfg->sample_rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                     I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .bclk = cfg->gpio_bclk,
            .ws   = cfg->gpio_ws,
            .dout = I2S_GPIO_UNUSED,
            .din  = cfg->gpio_din,
            .invert_flags = { .ws_inv = false },
        },
    };
    /* SPH0645 is left-channel; select left slot only */
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_chan, &std_cfg), TAG, "std init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_chan), TAG, "enable failed");

    ESP_LOGI(TAG, "I2S mic ready — %"PRIu32" Hz, window %"PRIu32" samples",
             cfg->sample_rate, cfg->sample_count);
    return ESP_OK;
}

esp_err_t i2s_mic_read_level(float *out_level)
{
    ESP_RETURN_ON_FALSE(s_rx_chan && s_buf && out_level, ESP_ERR_INVALID_STATE, TAG, "not initialised");

    size_t bytes_to_read = s_cfg.sample_count * sizeof(int32_t);
    size_t bytes_read    = 0;

    ESP_RETURN_ON_ERROR(
        i2s_channel_read(s_rx_chan, s_buf, bytes_to_read, &bytes_read, pdMS_TO_TICKS(1000)),
        TAG, "i2s read failed");

    uint32_t n = bytes_read / sizeof(int32_t);

    /* First pass: compute mean to remove DC offset (SPH0645 has significant DC bias) */
    double sum = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        sum += (double)(s_buf[i] >> SPH0645_SHIFT);
    }
    double mean = sum / n;

    /* Second pass: compute RMS of AC component */
    double sum_sq = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        double sample = (double)(s_buf[i] >> SPH0645_SHIFT) - mean;
        sum_sq += sample * sample;
    }
    double rms = sqrt(sum_sq / n);

    /* Convert to dBFS: 20*log10(rms / full_scale) */
    double full_scale = (double)(1 << 23);
    float linear = (float)(rms / full_scale);
    float dbfs = -100.0f;
    if (rms > 0.0) {
        dbfs = 20.0f * log10f(linear);
    }

    ESP_LOGI(TAG, "mean=%.0f rms=%.1f linear=%.6f dBFS=%.1f", mean, rms, linear, dbfs);

    *out_level = linear;
    if (*out_level > 1.0f) *out_level = 1.0f;

    return ESP_OK;
}

esp_err_t i2s_mic_deinit(void)
{
    if (s_rx_chan) {
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
    free(s_buf);
    s_buf = NULL;
    return ESP_OK;
}
