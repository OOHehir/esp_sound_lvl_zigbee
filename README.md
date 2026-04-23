# esp_sound_lvl_zigbee

ESP32-C6 Zigbee end device that reads ambient sound levels from a MEMS microphone via I2S and reports normalized RMS amplitude to Zigbee2MQTT.

## Key Technologies

- **MCU:** ESP32-C6 (native IEEE 802.15.4 radio)
- **Sensor:** SPH0645LM4H MEMS microphone (I2S, 16 kHz)
- **Protocol:** Zigbee Home Automation (0x0104), Analog Input cluster (0x000C)
- **Stack:** ESP-IDF 5.2+ + Espressif Zigbee SDK ≥1.3.0
- **Integration:** Zigbee2MQTT

## Signal Processing

Captures 1024-sample I2S frames at 16 kHz. Applies two-pass DC offset removal, then computes RMS amplitude normalized to [0.0, 1.0]. Reports on significant changes (delta ≥ 0.0005) with a 1 s minimum and 300 s maximum interval.

## Getting Started

**Prerequisites:** ESP-IDF 5.2+, ESP32-C6 devkit, SPH0645LM4H wired to I2S pins

```bash
idf.py set-target esp32c6
idf.py build flash monitor
```

Pair the device to your Zigbee network. The device appears as manufacturer "Electronics Consultants" on the Analog Input cluster in Zigbee2MQTT.

---

Built by Owen O'Hehir — embedded Linux, IoT, Matter & Rust consulting at [electronicsconsult.com](https://electronicsconsult.com). Available for contract and consulting work.
