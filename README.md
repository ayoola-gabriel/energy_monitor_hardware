# Energymeter

ESP32-based energy monitoring and load control project with local automation, BLE, and Supabase / cloud telemetry.

## Overview

This project collects power and environmental data from two loads:

- Air conditioner via `PZEM-004T` (voltage, current, power, energy, frequency, power factor)
- Light/bulb via `ACS712-30A` current sensor

It also reads:

- Ambient light with an `LDR`
- Occupancy with an `HC-SR501` PIR sensor
- Temperature and humidity with a `DHT11`

The ESP32 stores preferences in LittleFS, provides BLE telematics, and uploads telemetry data to a Supabase backend.

## Key features

- Automatic bulb and AC control based on occupancy, light level, temperature, and hysteresis
- BLE characteristics for settings, state, WiFi credentials, and telemetry
- Persistent device settings and state using LittleFS
- Automatic reconnection and Supabase telemetry upload
- Dual PZEM sensor support: one for AC and one for bulb
- 16x2 LCD display with rotating screens every 3 seconds
- Factory reset support via button hold

## Hardware

### Sensors and actuators

- `ACS712-30A` for bulb current sensing
- `PZEM-004T` for AC metering
- `PZEM-004T` for bulb metering
- `HC-SR501` PIR motion sensor
- `LDR` ambient light sensor
- `DHT11` temperature/humidity sensor
- `16x2 I2C LCD` via `PCF8574`
- LEDs for bulb and AC status
- Buttons for bulb control, AC control, and function/factory reset

### ESP32 wiring

- `D19` — PIR sensor input
- `D18` — DHT11 data
- `D36` — LDR input
- `D5` — bulb toggle button
- `D17` — function button
- `D16` — AC toggle button
- `D4` — bulb status LED
- `D2` — AC status LED
- `D33` — bulb control output
- `D12` — AC control output

### PZEM connections

- AC PZEM:
  - `D13` — RX
  - `D14` — TX
- Bulb PZEM:
  - `D27` — RX
  - `D26` — TX

### LCD I2C

- `SDA` to ESP32 SDA
- `SCL` to ESP32 SCL

## Software behavior

### Control logic

- Bulb turns on when occupancy is detected and it is dark, respecting hysteresis
- AC turns on when occupancy is detected and temperature exceeds threshold, with a delay between state changes
- Loads may be turned off if energy or other thresholds are exceeded

### Telemetry

- Sensor data is read once per second
- Totals and aggregated values are uploaded every minute
- If the network is unavailable, data is stored in LittleFS and uploaded later

### Preferences and persistence

The device saves:

- `ldr_threshold`
- `ldr_hysteresis`
- `temp_threshold`
- `temp_hysteresis`
- Load states
- WiFi credentials
- Daily energy/day tracking values

## Partition and LittleFS note

The project uses `LittleFS` for persistent storage, so your partition table must include a `littlefs` data partition.

In `platformio.ini` the board partition is configured with:

```ini
board_build.partitions = huge_app.csv
```

Make sure `huge_app.csv` contains a `littlefs` entry such as:

```csv
nvs,data,nvs,0x9000,0x6000
otadata,data,ota,0xF000,0x2000
app0,app,ota_0,0x10000,0x140000
app1,app,ota_1,0x150000,0x140000
littlefs,data,littlefs,0x290000,0x40000
```

Adjust offsets and sizes to fit your module flash memory.

## Build and upload

Use PlatformIO for build and upload:

```bash
pio run
pio run -t upload
```

## Dependencies

The project uses these PlatformIO libraries:

- `mathertel/LiquidCrystal_PCF8574`
- `adafruit/DHT sensor library`
- `mandulaj/PZEM-004T-v30`
- `jhagas/ESPSupabase`
- `bblanchon/ArduinoJson`
- `h2zero/NimBLE-Arduino`

## Notes

- The code currently uploads to Supabase rather than ThingSpeak
- WiFi credentials can be updated over BLE and are persisted in LittleFS
- The function button also supports factory reset when held during startup

## License

This project is licensed under the MIT License. See `LICENSE`.

