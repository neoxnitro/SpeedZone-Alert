# SpeedZone-Alert 🚗🚨

This repository is an ESP32-based GPS/zone monitoring project (TTGO-style boards). It provides GPS parsing, zone/radar detection, buzzer alerts and a simple display. The original README contained Arduino IDE-centric instructions and examples that are outdated for this codebase; this file has been updated to match the current PlatformIO-based project layout and the actual pin/configuration used in `src/main.cpp`.

## Features ✨

- Real-time GPS parsing using `TinyGPSPlus`
- Zone / radar detection and proximity alerts
- Two-pin buzzer output for audible alerts
- TFT display support for status

## Software Requirements 📚

### Build system (PlatformIO - recommended)

This project uses PlatformIO (VS Code) and is configured in `platformio.ini`.

Basic steps:

- Install Visual Studio Code and the PlatformIO extension (PlatformIO IDE).
- Open the repository folder in VS Code and use the PlatformIO sidebar to build and upload.
- From a terminal you can run:

```bash
pio run            # build
pio run -t upload  # upload to the connected board
pio device monitor 115200  # open serial monitor
```

The project includes `TinyGPSPlus` and other dependencies via PlatformIO; PlatformIO will fetch them automatically.

## Installation & Setup 🚀

### Step 1: Clone or Open the Project

Open this folder in VS Code with the PlatformIO extension, or clone the repo locally:

```bash
git clone <your-repo-url>
code <repo-folder>
```

### Step 2: Configure Your Setup

Configuration is done in the project source and headers (see `include/User_Setup.h` and `src/main.cpp`). The active pins used in this codebase (check `src/main.cpp`) are:

```cpp
// GPS (UART1)
static const int GPS_RX_PIN = 21;
static const int GPS_TX_PIN = 22;

// Buzzer (two-pin passive buzzer wiring)
static const int BUZZER_PIN_A = 25;
static const int BUZZER_PIN_B = 26;
```

If you change pins, update `include/User_Setup.h` or the pin constants in `src/main.cpp` accordingly and rebuild.

### Step 3: Build and Upload (PlatformIO)

From PlatformIO (VS Code):
- Use the PlatformIO toolbar to `Build` and `Upload`.

From the command line in the project root:

```bash
pio run
pio run -t upload
pio device monitor 115200
```

### Step 4: Verify Serial Output

Open a serial monitor at `115200` baud. Typical startup output includes initialization messages and GPS status.

## Wiring / Hardware Notes 🔧

ESP32          GPS NEO-6M
-----          ----------
3.3V    <-->   VCC
GND     <-->   GND
GPIO21  <-->   TX (GPS TX -> ESP32 RX)
GPIO22  <-->   RX (GPS RX -> ESP32 TX)

ESP32          Passive Buzzer (two-pin)
-----          --------------------------
GPIO25  <-->   Buzzer pin A
GPIO26  <-->   Buzzer pin B

Note: This code uses a two-pin (passive) buzzer method where the firmware toggles the two GPIOs in opposite phase. If you have an active buzzer (single-pin), adapt wiring and code accordingly.

## Zones / Radar definitions

This repository implements radar/zone detection logic (`include/radar.h`, `src/zone.cpp`). The previous README referenced a `cameras[]` array and an `.ino` sketch; this code stores and evaluates zones differently. To add or change detection zones or radars, inspect and edit the relevant files in `src/` and `include/`.

## Project Structure 📁

```
<project root>/
├── platformio.ini         # PlatformIO configuration
├── include/               # public headers (e.g. User_Setup.h, radar.h)
├── src/                   # implementation (main.cpp, radar.cpp, zone.cpp, drivers)
└── README.md              # This file
```

## Contributing 🤝

Contributions are welcome. Open issues or PRs for:
- Bug fixes
- New features
- Documentation improvements

If you change pin mappings or add region-specific zone data, please document it in the repository so others can reuse it.

## Safety Notice ⚠️

- Always keep attention on the road — this is an aid, not a substitute for safe driving.
- This project is provided as-is for educational use.

---

If you want, I can also:

- update the README language to French,
- add a short `README-fr.md` translation,
- or add a small example of how to provide zone data.

Tell me which you'd like next.
