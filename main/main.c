#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s_mic.h"
#include "zigbee_sensor.h"

static const char *TAG = "main";

/* ── Hardware config ─────────────────────────────────────────── */
#define MIC_GPIO_BCLK    0
#define MIC_GPIO_WS      1
#define MIC_GPIO_DIN     2
#define MIC_SAMPLE_RATE  16000
#define MIC_SAMPLE_COUNT 1024

#define REPORT_INTERVAL_S 5

/* ── Sound sampling task ─────────────────────────────────────── */
static void sound_task(void *arg)
{
    float level = 0.0f;
    while (1) {
        if (i2s_mic_read_level(&level) == ESP_OK) {
            ESP_LOGI(TAG, "Sound level: %.4f", level);
            zigbee_sensor_set_level(level);
        }
        vTaskDelay(pdMS_TO_TICKS(REPORT_INTERVAL_S * 1000));
    }
}

/* ── app_main ────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Initialise I2S microphone */
    i2s_mic_config_t mic_cfg = {
        .gpio_bclk    = MIC_GPIO_BCLK,
        .gpio_ws      = MIC_GPIO_WS,
        .gpio_din     = MIC_GPIO_DIN,
        .sample_rate  = MIC_SAMPLE_RATE,
        .sample_count = MIC_SAMPLE_COUNT,
    };
    ESP_ERROR_CHECK(i2s_mic_init(&mic_cfg));

    /* Start sound task immediately */
    xTaskCreate(sound_task, "sound", 4096, NULL, 5, NULL);

    /* Start Zigbee (joins network in background) */
    zigbee_sensor_config_t zb_cfg = {
        .endpoint = 1,
        .on_joined = NULL,
    };
    ESP_ERROR_CHECK(zigbee_sensor_start(&zb_cfg));
}
