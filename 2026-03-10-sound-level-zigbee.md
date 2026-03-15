# ESP32-C6 Sound Level Monitor

Read SPH0645LM4H sound level via I2S on ESP32-C6 and report normalised RMS amplitude (0.0-1.0) over Zigbee using the Analog Input cluster (`0x000C`, `present_value` attribute `0x0055`).

## Project Structure

```
├── CMakeLists.txt
├── sdkconfig.defaults
├── partitions.csv                # Custom: zb_storage + zb_fct partitions
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml         # esp-zigbee-lib + esp-zboss-lib
│   └── main.c                    # Init mic, start sound task, start zigbee
└── components/
    ├── i2s_mic/                  # I2S mic driver + DC offset removal + RMS calc
    │   ├── include/i2s_mic.h
    │   └── i2s_mic.c
    └── zigbee_sensor/            # Zigbee End Device + Analog Input cluster
        ├── include/zigbee_sensor.h
        └── zigbee_sensor.c
```

## Hardware Wiring (SPH0645LM4H -> ESP32-C6)

| Mic Pin | ESP32-C6 | I2S Signal |
|---------|----------|------------|
| VDD     | 3.3V     | —          |
| GND     | GND      | —          |
| SCK     | GPIO 3   | I2S_BCK    |
| LR      | GPIO 1   | I2S_WS     |
| SD      | GPIO 2   | I2S_DIN    |
| LR      | GND      | Left chan  |

## Build & Flash

```bash
source /home/claude/.espressif/tools/activate_idf_v5.5.3.sh
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Monitor (no TTY in VM):
```bash
stty -F /dev/ttyACM0 115200 raw -echo; timeout 30 cat /dev/ttyACM0
```

## Zigbee Details

- **Device type:** Zigbee End Device (ZED)
- **Profile:** Home Automation (`0x0104`)
- **Cluster:** Analog Input (`0x000C`), server role
- **Attribute:** `present_value` (`0x0055`) — single-precision float, 0.0-1.0
- **Report interval:** 5 seconds (configurable via `REPORT_INTERVAL_S` in `main.c`)

## Signal Processing

- SPH0645 outputs 24-bit left-justified in 32-bit I2S slots
- DC offset removal: two-pass (mean subtraction then RMS)
- Normalisation: RMS / 2^23 (full-scale 24-bit), clamped to [0.0, 1.0]
- Logs both linear level and dBFS for diagnostics

## Coordinator / Data Collection (RPi v4)

See `/home/claude/projects/zigbee2mqtt/README.md` for full documentation.

- **SONOFF Zigbee 3.0 USB Dongle Plus V2** as coordinator
- **zigbee2mqtt** (Docker) with custom converter (`converters/sound_monitor.js`)
- **Mosquitto** (Docker) MQTT broker
- **Python collector** — stores numeric attributes in SQLite (EAV schema)

## Notes

- **Power saving:** Add `esp_zb_sleep_enable()` for battery operation once functional
- **Isolation testing:** Comment out `zigbee_sensor_start()` in `app_main` to test mic over serial without a coordinator
