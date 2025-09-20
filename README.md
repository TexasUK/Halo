# HALO Strobe Controller (ESP32-S3) v0.4.0

**HALO** is a compact ESP32-S3 flight/traffic companion with a TFT UI, audio prompts, and MOSFET-driven strobe control. It ingests FLARM/SoftRF NMEA over UART, renders a minimal UI, speaks traffic alerts, and adjusts strobe cadence by alert level. A small BLE control plane lets you tweak settings and run a bench test sequence.

## Hardware

- **MCU**: ESP32-S3 (tested on LOLIN S3 Mini / esp32-s3-zero)
- **Display**: ST7735 160×128 TFT (SPI)
- **Barometer**: BMP280 (I²C)
- **Audio**: DFPlayer Mini (TX only + BUSY)
- **Navigation Input**: FLARM / SoftRF (UART RX)
- **Lighting**: Strobe MOSFET (digital out)

![IMG_20250920_131226](https://github.com/user-attachments/assets/839cd77a-67d7-476b-8b7b-fd425f63677d)

### Pin Map (Current Build)

```
I2C          SDA = 4,  SCL = 5
TFT          SCLK=1, MOSI=2, MISO=-1, CS=10, DC=11, RST=13, BL=3 (PWM)
DFPlayer     TX=9 (ESP→DF), BUSY=7 (active LOW)
FLARM/SoftRF RX=8 (UART2 RX)
STROBE       gate=6
```

## Features

### Boot & Pre-Flight
- Splash screen (RGB565 in PROGMEM) → version card → Pre-Flight page
- Boot chime (track 1) after initialization completes
- Pre-Flight displays: Temperature (°C), QNH (hPa), Airfield Elevation (ft), Volume

### Cruise Mode
- Compass tape with labels every 45°
- Heading chevron and tiny degree dot marker
- Bottom line: speed (kts) left, altitude (ft) right

![IMG_20250920_131533](https://github.com/user-attachments/assets/2753fdca-f0b5-4439-a155-b0a2bf2d5b2b)

### Traffic (Alert View)
- **Top line**: distance (km, 1dp) left · Δalt (ft) centered · bearing (°) right
- **Ring display**: center tint by alert level (L1 green / L2 amber / L3 red)
- **Glider glyph** in center, target dot scaled by range (clamped)
- **Bearing arrow** slightly outside the ring
- **Vertical indicator** at right (arrow up/down if |Δalt|>200 ft, dot if level)
- 
![IMG_20250920_131542](https://github.com/user-attachments/assets/1359676f-4a7d-44f6-9b6b-8163bc064058)

### Landing / Landed
- **Landing**: Large values for Speed (kts) and Altitude (ft)
- **Landed**: Duration (HH:MM), UTC Time (HH:MM), Alerts (count)

## Audio System (DFPlayer)

### Spoken Alerts
Two-part alert system:
- **Vertical**: 10=LEVEL, 11=HIGH, 12=LOW
- **Clock position**: 21..32 for 1..12 o'clock

## Strobe Control

- **Status**: ON while FLYING/ALERT; OFF in LANDING/LANDED and at panic reset
- **Base timing**: on=120ms

### Cadence by Alert Level
- **L1**: period 1400ms
- **L2**: period 900ms  
- **L3**: period 500ms
- **Standard**: on=120ms, period=2000ms (when alert clears)

## Navigation Input

**UART2 (RX)** with configurable baud rates:
- **FLARM**: 19200 baud
- **SoftRF**: 38400 baud

Parses NMEA sentences: RMC, GGA, PFLAA
- Updates telemetry (SOG, track, altitude, UTC)
- Updates alert snapshot (bearing/range/vertical)
- `navValid()` drives the FLARM badge

## Flight State Machine (FSM)

### State Transitions

**PREFLIGHT → FLYING**
- SOG > `TAKEOFF_KTS` held for `TAKEOFF_HOLD_MS`, OR
- AGL > `TAKEOFF_ALT_FT` with same hold

**FLYING → ALERT**  
- New alert detected (with minimum `TRAFFIC` hold)

**FLYING/ALERT → LANDING**
- AGL ≤ `LANDING_ALT_FT` for 2 seconds
- Strobes turn off, landing audio plays

**LANDING → LANDED**
- SOG < 5 kts for 3 seconds
- Shows landed screen and records flight stats

## Storage (NVS)

### Settings (Load/Save)
- `qnh_hPa`: QNH pressure setting
- `airfieldElev_ft`: Airfield elevation
- `volume0_30`: Audio volume (0-30)
- Baseline AGL (`baselineSet`/`baselineAlt_m`)
- `data_source`: FLARM or SoftRF selection

### Flight Records
Recorded at LANDED:
- Flight duration (ms)
- Alert count  
- UTC timestamp (hh:mm)

## Console Test Commands

Connect via Serial at **115200 baud**:

| Key | Action |
|-----|--------|
| `J` | Play track 3 (audio test) |
| `T` | Force FLYING state via FSM (plays track 3) |
| `1`/`2`/`3` | Trigger alert L1/L2/L3 (Traffic view, speak vertical then 2 o'clock) |
| `R` | Capture baseline AGL now and persist |
| `L` | Force Landing (plays track 7), then LANDED once <5 kts for 3s |
| `C` | **PANIC**: Stop audio, clear alerts, strobes off, FSM reset, return to BOOT |

## BLE Control Interface

Halo controller app details soon to be released

## Build & Installation

### Requirements
- **PlatformIO** project (`platformio.ini` included)
- **Framework**: Arduino (ESP32-S3 core)

### Dependencies
- Adafruit GFX
- ST7735 driver
- BMP280 library  
- ESP32 Arduino BLE stack
- DFPlayer handled via project's `drivers/dfplayer.*`

### Configuration
- **Serial**: 115200 baud
- **TFT**: Initialized with `INITR_GREENTAB` (adjust if your panel variant differs)

## Code Structure

```
src/
├── main.cpp                    // UI, rendering, splash, keys, strobe driver, boot audio, BLE hooks
├── app/
│   ├── app_fsm.h/.cpp         // FSM: states, guards, cadence, NVS flight record
│   └── telemetry.h            // Runtime telemetry (SOG, track, alt, UTC, etc.)
├── nav/
│   └── flarm.h/.cpp           // UART ingest + NMEA (RMC/GGA/PFLAA) parsing  
├── drivers/
│   └── dfplayer.h/.cpp        // DFPlayer Mini helpers (queue & play)
├── storage/
│   └── nvs_store.h/.cpp       // Settings load/save; nvs_record_flight()
├── ble/
│   └── ble_ctrl.h/.cpp        // BLE service + characteristics, parsing & persistence
├── ui_iface.h                 // Page enum + ui_set_page bridge
├── constants.h, policy.h      // Tunables (takeoff/landing thresholds, alert holds)
├── events.h                   // Event definitions/hooks
└── splash_image.cpp           // PROGMEM RGB565 splash
```

## Design Notes

- **Change-only rendering** to avoid flicker on ST7735
- **Watchdog-friendly splash** with periodic `yield()` during blit operations  
- **Hard reset key (C)** centralizes "get me out of any bench mess" behavior
- **BLE writes** normalize and persist immediately; readbacks echo stored controller values
- **Bench TEST** extends landing inhibit during test steps; lands once, then stops

## License

GNU GENERAL PUBLIC LICENSE
Version 3, 29 June 2007

## Contributing

Happy for anyone who knows what they are doing to suggest improvements.
