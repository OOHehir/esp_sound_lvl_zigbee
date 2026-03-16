#include "zigbee_sensor.h"
#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "zigbee_sensor";

#define ZB_ANALOG_IN_CLUSTER   0x000C
#define ZB_ATTR_PRESENT_VALUE  0x0055

static zigbee_sensor_config_t s_cfg = {0};
static bool s_joined = false;

/* ── Configure attribute reporting on the stack ────────────── */
static void configure_reporting(void)
{
    esp_zb_zcl_reporting_info_t info = {
        .direction   = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep          = s_cfg.endpoint,
        .cluster_id  = ZB_ANALOG_IN_CLUSTER,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id     = ZB_ATTR_PRESENT_VALUE,
        .u.send_info = {
            .min_interval     = 1,      /* don't report faster than 1 s  */
            .max_interval     = 300,    /* report at least every 5 min   */
            .def_min_interval = 1,
            .def_max_interval = 300,
            .delta.s32        = 0,      /* we handle change threshold in app */
        },
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .manuf_code  = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_err_t err = esp_zb_zcl_update_reporting_info(&info);
    ESP_LOGI(TAG, "Reporting configured for analog_input.present_value: %s",
             esp_err_to_name(err));
}

/* ── Zigbee stack signal handler (extern, called by stack) ─── */
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
                configure_reporting();
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGW(TAG, "Zigbee init failed (status %d)", status);
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (status == ESP_OK) {
                ESP_LOGI(TAG, "Network joined successfully");
                s_joined = true;
                if (s_cfg.on_joined) {
                    s_cfg.on_joined();
                }
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
    return ESP_OK;
}

/* ── Zigbee task ─────────────────────────────────────────────── */
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

    /* Basic cluster with manufacturer + model for z2m identification */
    esp_zb_basic_cluster_cfg_t basic_cfg = { .power_source = 0x01 };
    esp_zb_attribute_list_t *basic_attrs = esp_zb_basic_cluster_create(&basic_cfg);
    static char manufacturer[] = {18, 'E', 'l', 'e', 'c', 't', 'r', 'o', 'n', 'i', 'c', 's', 'C', 'o', 'n', 's', 'u', 'l', 't'};
    esp_zb_basic_cluster_add_attr(basic_attrs,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    static char model[] = {14, 's', 'o', 'u', 'n', 'd', '-', 'l', 'e', 'v', 'e', 'l', '-', 'v', '1'};
    esp_zb_basic_cluster_add_attr(basic_attrs,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);

    esp_zb_identify_cluster_cfg_t identify_cfg = { .identify_time = 0 };

    /* Analog Input cluster with present_value attribute */
    esp_zb_analog_input_cluster_cfg_t ai_cfg = {
        .present_value = 0.0f,
    };
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list,
        esp_zb_identify_cluster_create(&identify_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_input_cluster(
        cluster_list,
        esp_zb_analog_input_cluster_create(&ai_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint       = s_cfg.endpoint,
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

/* ── Public API ──────────────────────────────────────────────── */
esp_err_t zigbee_sensor_start(const zigbee_sensor_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;

    BaseType_t ret = xTaskCreate(zb_task, "zigbee", 8192, NULL, 6, NULL);
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}

esp_err_t zigbee_sensor_set_level(float level)
{
    if (!s_joined) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_zb_lock_acquire(portMAX_DELAY);
    /* notify=true triggers the stack's reporting mechanism from the ZB task context */
    esp_err_t err = esp_zb_zcl_set_attribute_val(
        s_cfg.endpoint,
        ZB_ANALOG_IN_CLUSTER,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ATTR_PRESENT_VALUE,
        &level, true);
    esp_zb_lock_release();
    return err;
}
