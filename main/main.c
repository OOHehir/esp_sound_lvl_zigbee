#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s_mic.h"

static const char *TAG = "main";

/* ── I2S config ───────────────────────────────────────────────── */
#define MIC_GPIO_BCLK    6
#define MIC_GPIO_WS      5
#define MIC_GPIO_DIN     4
#define MIC_SAMPLE_RATE  16000
#define MIC_SAMPLE_COUNT 1024

/* ── Zigbee endpoint / cluster IDs ───────────────────────────── */
#define ZB_ENDPOINT_ID          1
#define ZB_ANALOG_IN_CLUSTER    0x000C
#define ZB_ATTR_PRESENT_VALUE   0x0055
#define ZB_REPORT_INTERVAL_S    5

/* ── Forward declarations ────────────────────────────────────── */
static void zb_task(void *arg);

/* ── Sound sampling task ─────────────────────────────────────── */
static void sound_task(void *arg)
{
    float level = 0.0f;
    while (1) {
        if (i2s_mic_read_level(&level) == ESP_OK) {
            ESP_LOGI(TAG, "Sound level: %.3f", level);

            /* Update Zigbee attribute — ZCL single (4-byte float) */
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_set_attribute_val(ZB_ENDPOINT_ID,
                                         ZB_ANALOG_IN_CLUSTER,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                         ZB_ATTR_PRESENT_VALUE,
                                         &level, false);
            esp_zb_lock_release();
        }
        vTaskDelay(pdMS_TO_TICKS(ZB_REPORT_INTERVAL_S * 1000));
    }
}

/* ── Zigbee stack callbacks ──────────────────────────────────── */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *sig = signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;

    switch (*sig) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialised");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (status == ESP_OK) {
                ESP_LOGI(TAG, "Zigbee stack ready — joining network");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGW(TAG, "Zigbee init failed (status %d)", status);
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (status == ESP_OK) {
                ESP_LOGI(TAG, "Network joined successfully");
                xTaskCreate(sound_task, "sound", 4096, NULL, 5, NULL);
            } else {
                ESP_LOGW(TAG, "Network steering failed (status %d), retrying...", status);
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            break;
        default:
            ESP_LOGD(TAG, "ZDO signal: %d, status: %d", *sig, status);
            break;
    }
}

/* ── Zigbee action handler ───────────────────────────────────── */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    /* Device is report-only — no inbound actions to handle */
    return ESP_OK;
}

/* ── Zigbee initialisation task ─────────────────────────────── */
static void zb_task(void *arg)
{
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = 3000,
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    /* Analog Input cluster with present_value attribute */
    esp_zb_analog_input_cluster_cfg_t ai_cfg = {
        .present_value = 0.0f,
    };
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_analog_input_cluster(
        cluster_list,
        esp_zb_analog_input_cluster_create(&ai_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint       = ZB_ENDPOINT_ID,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg);
    esp_zb_device_register(ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/* ── app_main ────────────────────────────────────────────────── */
void app_main(void)
{
    /* NVS required by Zigbee stack for commissioning data */
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

    /* Zigbee runs in its own high-priority task (required by SDK) */
    xTaskCreate(zb_task, "zigbee", 8192, NULL, 6, NULL);
}
