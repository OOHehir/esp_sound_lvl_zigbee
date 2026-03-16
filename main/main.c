#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s_mic.h"
#include "zigbee_sensor.h"

static const char *TAG = "main";

/* ── Hardware config ─────────────────────────────────────────── */
#define MIC_GPIO_BCLK    3
#define MIC_GPIO_WS      1
#define MIC_GPIO_DIN     2
#define MIC_SAMPLE_RATE  16000
#define MIC_SAMPLE_COUNT 1024

#define SAMPLE_INTERVAL_S  1
#define CHANGE_THRESHOLD   0.0005f

#define ZB_RESET_GPIO      GPIO_NUM_22  /* Pull-down; HIGH on boot = factory reset */

/* ── Sound sampling task ─────────────────────────────────────── */
static void sound_task(void *arg)
{
    float level = 0.0f;
    float prev_level = 0.0f;
    while (1) {
        if (i2s_mic_read_level(&level) == ESP_OK) {
            float delta = level - prev_level;
            if (delta < 0) delta = -delta;
            if (delta >= CHANGE_THRESHOLD) {
                ESP_LOGI(TAG, "Sound level: %.4f (delta=%.4f)", level, delta);
                zigbee_sensor_set_level(level);
                prev_level = level;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_S * 1000));
    }
}

/* ── Zigbee factory reset check ──────────────────────────────── */
static bool check_zigbee_reset(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << ZB_RESET_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&cfg);
    vTaskDelay(pdMS_TO_TICKS(50));  /* let level settle */

    if (gpio_get_level(ZB_RESET_GPIO) == 1) {
        ESP_LOGW(TAG, "GPIO %d HIGH — erasing Zigbee storage + NVS for factory reset", ZB_RESET_GPIO);
        const esp_partition_t *zb_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_ANY, "zb_storage");
        if (zb_part) {
            esp_partition_erase_range(zb_part, 0, zb_part->size);
        }
        nvs_flash_erase();
        return true;
    }
    return false;
}

/* ── app_main ────────────────────────────────────────────────── */
void app_main(void)
{
    bool factory_reset = check_zigbee_reset();

    esp_err_t ret = nvs_flash_init();
    if (factory_reset || ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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
