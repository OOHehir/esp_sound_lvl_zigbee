# ESP32-C6 Sound Level Monitor — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Read SPH0645LM4H sound level via I²S on ESP32-C6 and report RMS amplitude over Zigbee.

**Architecture:** A self-contained `components/i2s_mic` component handles all hardware interaction and exposes a clean API. `main.c` wires up the Zigbee stack and polls the component for level readings. Zigbee reporting uses the Espressif esp-zigbee-sdk, exposing a custom or analog-input cluster attribute.

**Tech Stack:** ESP-IDF (IDF Component Manager v0.8), esp-zigbee-sdk (Espressif Zigbee component), C17, CMake.

to activate the environment, run the following command in your terminal:
       source "/home/claude/.espressif/tools/activate_idf_v5.5.3.sh"

---

## Project Structure

```
sound_monitor/
├── CMakeLists.txt                  # top-level
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   └── main.c                      # minimal: init + zigbee event loop
└── components/
    └── i2s_mic/
        ├── CMakeLists.txt
        ├── include/
        │   └── i2s_mic.h           # public API
        └── i2s_mic.c               # I²S init, DMA read, RMS calculation
```

---

## SPH0645LM4H Wiring (ESP32-C6)

| Mic Pin | ESP32-C6 GPIO | I²S Signal   |
|---------|---------------|--------------|
| VDD     | 3.3V          | —            |
| GND     | GND           | —            |
| BCLK    | GPIO 6        | I²S_BCK      |
| DOUT    | GPIO 4        | I²S_DIN      |
| LRCL    | GPIO 5        | I²S_WS       |
| SEL     | GND           | Left channel |

---

## Task 1: Project Scaffold

**Files:**
- Create: `CMakeLists.txt`
- Create: `sdkconfig.defaults`
- Create: `main/CMakeLists.txt`
- Create: `main/main.c` (stub only)
- Create: `components/i2s_mic/CMakeLists.txt`
- Create: `components/i2s_mic/include/i2s_mic.h`
- Create: `components/i2s_mic/i2s_mic.c` (stub only)

**Step 1: Create top-level CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sound_monitor)
```

**Step 2: Create sdkconfig.defaults**

```
CONFIG_ESP_ZIGBEE_ENABLED=y
CONFIG_ZB_ENABLED=y
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_160=y
```

**Step 3: Create main/CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES i2s_mic esp-zigbee-lib
)
```

**Step 4: Create components/i2s_mic/CMakeLists.txt**

```cmake
idf_component_register(
    SRCS "i2s_mic.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2s
)
```

**Step 5: Verify it compiles as empty project**

```bash
idf.py build
```
Expected: Build succeeds (no source errors yet).

**Step 6: Commit**

```bash
git add .
git commit -m "chore: project scaffold with i2s_mic component skeleton"
```

---

## Task 2: i2s_mic Component — Header (Public API)

**Files:**
- Modify: `components/i2s_mic/include/i2s_mic.h`

**Step 1: Write the header**

```c
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
```

**Step 2: Commit**

```bash
git add components/i2s_mic/include/i2s_mic.h
git commit -m "feat(i2s_mic): public API header"
```

---

## Task 3: i2s_mic Component — Implementation

**Files:**
- Modify: `components/i2s_mic/i2s_mic.c`

**Step 1: Write implementation**

```c
#include "i2s_mic.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_check.h"
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
            .invert_flags = { .ws_pol = false },
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

    /* Compute RMS over the window */
    double sum_sq = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        int32_t sample = s_buf[i] >> SPH0645_SHIFT;  /* 24-bit signed */
        sum_sq += (double)sample * sample;
    }
    double rms = sqrt(sum_sq / n);

    /* Normalise: SPH0645 24-bit, so max positive = 2^23 - 1 */
    *out_level = (float)(rms / ((double)(1 << 23)));
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
```

**Step 2: Build to catch compile errors**

```bash
idf.py build
```
Expected: Clean build.

**Step 3: Commit**

```bash
git add components/i2s_mic/i2s_mic.c
git commit -m "feat(i2s_mic): I2S init, DMA read, RMS level calculation"
```

---

## Task 4: Zigbee Plumbing — main.c

**Files:**
- Modify: `main/main.c`

**Notes on Zigbee approach:**
- Use `esp-zigbee-sdk` Analog Input (Basic) cluster (cluster ID `0x000C`) — this is the standard ZCL cluster for reporting a floating-point sensor value. Most Zigbee coordinators (Home Assistant ZHA/Zigbee2MQTT) can auto-discover it.
- The `present_value` attribute (0x0055) holds a `single precision float` — perfect for a 0.0–1.0 level.
- Device type: `ZED` (Zigbee End Device) — sleeps between reports to save power.

**Step 1: Write main.c**

```c
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
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_params_t *p);

/* ── Sound sampling task ─────────────────────────────────────── */
static void sound_task(void *arg)
{
    float level = 0.0f;
    while (1) {
        if (i2s_mic_read_level(&level) == ESP_OK) {
            ESP_LOGI(TAG, "Sound level: %.3f", level);

            /* Update Zigbee attribute — ZCL single (4-byte float) */
            esp_zb_zcl_attr_t attr = {
                .id         = ZB_ATTR_PRESENT_VALUE,
                .type       = ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                .access     = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
                .data.ptr   = &level,
            };
            esp_zb_zcl_set_attribute_val(ZB_ENDPOINT_ID,
                                         ZB_ANALOG_IN_CLUSTER,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                         ZB_ATTR_PRESENT_VALUE,
                                         &level, false);
        }
        vTaskDelay(pdMS_TO_TICKS(ZB_REPORT_INTERVAL_S * 1000));
    }
}

/* ── Zigbee stack callbacks ──────────────────────────────────── */
static void esp_zb_app_signal_handler(uint8_t bufid)
{
    esp_zb_app_signal_type_t *sig = esp_zb_app_signal_get(bufid, NULL);
    switch (*sig) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            ESP_LOGI(TAG, "Zigbee stack ready — joining network");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            ESP_LOGI(TAG, "Network joined successfully");
            xTaskCreate(sound_task, "sound", 4096, NULL, 5, NULL);
            break;
        default:
            break;
    }
    esp_zb_app_signal_process(bufid);
}

/* ── Zigbee initialisation task ─────────────────────────────── */
static void zb_task(void *arg)
{
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

    esp_zb_core_action_handler_register(zb_attribute_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();  /* never returns */
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_params_t *p)
{
    /* Nothing to handle inbound — device is report-only */
    return ESP_OK;
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
```

**Step 2: Build**

```bash
idf.py build
```

**Step 3: Commit**

```bash
git add main/main.c main/CMakeLists.txt
git commit -m "feat(main): zigbee ZED with analog input cluster + sound polling"
```

---

## Task 5: idf_component.yml — Declare Dependencies

**Files:**
- Create: `main/idf_component.yml`

**Step 1: Write idf_component.yml**

```yaml
## IDF Component Manager dependencies
dependencies:
  idf: ">=5.2.0"
  espressif/esp-zigbee-lib: ">=1.3.0"
  espressif/esp-zigbee-sdk: ">=1.3.0"
```

**Step 2: Fetch dependencies**

```bash
idf.py update-dependencies
idf.py build
```
Expected: Component manager downloads esp-zigbee packages and build succeeds.

**Step 3: Commit**

```bash
git add main/idf_component.yml
git commit -m "chore: add IDF component manager dependencies"
```

---

## Task 6: Flash and Verify

**Step 1: Flash to device**

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

**Step 2: Expected log output**

```
I (xxx) i2s_mic: I2S mic ready — 16000 Hz, window 1024 samples
I (xxx) main: Zigbee stack ready — joining network
I (xxx) main: Network joined successfully
I (xxx) main: Sound level: 0.012
I (xxx) main: Sound level: 0.031
```

**Step 3: Verify in coordinator**

In Home Assistant (ZHA) or Zigbee2MQTT, the device should appear as a simple sensor exposing `present_value` on the Analog Input cluster. The value updates every 5 seconds.

---

## Notes & Next Steps

- **Decibel conversion:** To report dBFS instead of linear RMS, replace `*out_level = rms / (1<<23)` with `*out_level = 20.0f * log10f(rms / (1<<23))` and change the ZCL attribute type to reflect range (typically –90 to 0 dBFS).
- **Reporting interval:** Adjust `ZB_REPORT_INTERVAL_S` freely — Zigbee binding/reporting attributes can also be configured from the coordinator side.
- **Power saving:** Add `esp_zb_sleep_enable()` and configure the poll rate for battery operation once functional.
- **Testing i2s_mic in isolation:** Before Zigbee, comment out `zb_task` in `app_main` and just run `sound_task` — this lets you validate mic levels over serial without needing a Zigbee coordinator.
