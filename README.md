# SpeedZone-Alert 🚗🚨

**GeoSpeed-Sentinel: Your Intelligent Road Safety Assistant**

An ESP32-based road safety system that uses GPS technology (u-blox NEO-6M) to monitor your location in real-time and alert you when approaching speed cameras. The system cross-references your coordinates with a fixed-speed camera database and provides intelligent proximity alerts through an active buzzer.

## Features ✨

- **Real-time GPS Tracking**: Continuous location monitoring using u-blox NEO-6M GPS module
- **Speed Camera Database**: Pre-configured database with camera locations and speed limits
- **Intelligent Proximity Alerts**: Two-tier warning system (warning zone and danger zone)
- **Speed Threshold Detection**: Automatic detection when exceeding speed limits near cameras
- **Active Buzzer Alerts**: Different beep patterns for varying alert levels
- **Serial Monitor Feedback**: Detailed status information and warnings
- **Haversine Distance Calculation**: Accurate distance computation between coordinates

## Hardware Requirements 🔧

### Components Needed:
1. **ESP32 Development Board** (any variant)
2. **u-blox NEO-6M GPS Module** with antenna
3. **Active Buzzer** (5V or 3.3V compatible)
4. **Jumper Wires**
5. **USB Cable** (for programming and power)
6. **Optional**: External power supply for vehicle use

### Wiring Diagram

```
ESP32          GPS NEO-6M
-----          ----------
3.3V    <-->   VCC
GND     <-->   GND
GPIO16  <-->   TX
GPIO17  <-->   RX

ESP32          Active Buzzer
-----          -------------
GPIO5   <-->   Signal/I/O
GND     <-->   GND
```

**Important Notes:**
- GPS module typically requires 3.3V (check your module specifications)
- Ensure proper antenna placement for GPS signal reception
- Active buzzer has polarity - connect correctly
- For vehicle use, consider using a voltage regulator for stable power

## Software Requirements 📚

### Arduino IDE Setup

1. **Install Arduino IDE** (version 1.8.x or 2.x)
   - Download from: https://www.arduino.cc/en/software

2. **Install ESP32 Board Support**
   - Open Arduino IDE
   - Go to `File` → `Preferences`
   - Add this URL to "Additional Board Manager URLs":
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to `Tools` → `Board` → `Boards Manager`
   - Search for "esp32" and install "esp32 by Espressif Systems"

3. **Install Required Library**
   - Go to `Sketch` → `Include Library` → `Manage Libraries`
   - Search for "TinyGPSPlus" by Mikal Hart
   - Click Install

## Installation & Setup 🚀

### Step 1: Clone or Download
```bash
git clone https://github.com/neoxnitro/SpeedZone-Alert.git
```

### Step 2: Configure Your Setup

1. Open `config.h` in Arduino IDE
2. Adjust pin configurations if using different pins:
   ```cpp
   #define GPS_RX_PIN 16        // ESP32 pin connected to GPS TX
   #define GPS_TX_PIN 17        // ESP32 pin connected to GPS RX
   #define BUZZER_PIN 5         // ESP32 pin connected to buzzer
   ```

3. Customize alert distances (in meters):
   ```cpp
   #define WARNING_DISTANCE_M 500  // Warning zone
   #define DANGER_DISTANCE_M 200   // Danger zone
   ```

### Step 3: Add Speed Camera Locations

1. Open `SpeedZone_Alert.ino`
2. Locate the `cameras[]` array
3. Add your local speed camera locations:
   ```cpp
   SpeedCamera cameras[] = {
     {"Camera Name", latitude, longitude, speedLimit},
     // Add more cameras here
   };
   ```

**Example:**
```cpp
{"Main St Camera", 40.7128, -74.0060, 50},
{"Highway 101", 37.7749, -122.4194, 80},
```

**Finding Coordinates:**
- Use Google Maps: Right-click location → Click coordinates to copy
- Use GPS coordinates in decimal format (not DMS)

### Step 4: Upload to ESP32

1. Connect ESP32 to computer via USB
2. In Arduino IDE:
   - Select `Tools` → `Board` → `ESP32 Dev Module` (or your board variant)
   - Select `Tools` → `Port` → [Your ESP32 Port]
3. Click Upload button (→)
4. Wait for upload to complete

### Step 5: Testing

1. Open Serial Monitor (`Tools` → `Serial Monitor`)
2. Set baud rate to **115200**
3. System should display:
   ```
   === SpeedZone Alert System ===
   Initializing...
   System Ready!
   Monitoring X speed camera locations
   Waiting for GPS fix...
   ```
4. Take device outside or near window for GPS signal
5. Wait for GPS fix (may take 30-60 seconds initially)

## Usage 📱

### Normal Operation

Once GPS has a fix, the system will:

1. **Continuously monitor your location**
2. **Calculate distance to all speed cameras**
3. **Provide status updates every 5 seconds** showing:
   - Current position
   - Current speed
   - Nearest camera information
   - Distance to nearest camera

### Alert Zones

**Warning Zone (500m - 200m from camera):**
- 🟡 Buzzer beeps every 2 seconds
- Serial monitor shows: `** Warning - Speed Camera Ahead **`

**Danger Zone (within 200m of camera):**
- 🔴 Buzzer beeps rapidly every 0.5 seconds
- Serial monitor shows: `*** DANGER ZONE - SLOW DOWN! ***`
- If exceeding speed limit: `!!! SPEED LIMIT EXCEEDED !!!`

### Serial Monitor Output Example

```
--- Status Update ---
Position: 40.712345, -74.006789
Speed: 55.3 km/h
Nearest Camera: Camera 1 - Main Street
Distance: 350 m
Speed Limit: 50 km/h
** Warning - Speed Camera Ahead **
```

## Configuration Options ⚙️

### Customizing Alert Distances

Edit in `config.h`:
```cpp
#define WARNING_DISTANCE_M 500   // Adjust warning distance
#define DANGER_DISTANCE_M 200    // Adjust danger distance
```

### Customizing Beep Patterns

Edit in `config.h`:
```cpp
#define WARNING_BEEP_INTERVAL 2000  // Milliseconds between beeps
#define DANGER_BEEP_INTERVAL 500    // Rapid beeping interval
#define BEEP_DURATION 100           // How long each beep lasts
```

## Troubleshooting 🔍

### GPS Not Getting Fix
- Ensure GPS module has clear view of sky
- GPS requires outdoor operation or near window
- First fix may take 30-60 seconds (cold start)
- Check wiring connections
- Verify GPS TX → ESP32 RX16 and GPS RX → ESP32 TX17

### No Serial Output
- Check baud rate is set to 115200
- Verify USB connection
- Check correct COM port is selected

### Buzzer Not Working
- Verify buzzer is an **active** buzzer (has internal oscillator)
- Check polarity of buzzer connection
- Verify GPIO5 connection

### GPS Shows "No GPS data received"
- Check GPS module power (3.3V)
- Verify RX/TX connections (crossed correctly)
- Test GPS module with separate GPS test sketch

## Project Structure 📁

```
SpeedZone-Alert/
│
├── SpeedZone_Alert.ino    # Main Arduino sketch
├── config.h               # Configuration file
└── README.md             # This file
```

## How It Works 🧠

1. **GPS Module** continuously receives satellite signals and provides:
   - Current latitude/longitude
   - Current speed
   - Time and date

2. **ESP32** processes GPS data using TinyGPS++ library:
   - Parses NMEA sentences
   - Extracts location and speed information

3. **Distance Calculation** uses Haversine formula:
   - Calculates great-circle distance between two points on Earth
   - Accurate for distances up to several kilometers

4. **Proximity Detection**:
   - Compares current location to each camera in database
   - Determines nearest camera and distance
   - Triggers appropriate alert based on distance thresholds

5. **Alert System**:
   - Controls buzzer with different patterns
   - Provides visual feedback via Serial Monitor
   - Monitors speed relative to camera speed limits

## Customization Ideas 💡

### Expand the Database
- Add more speed camera locations
- Include different types of enforcement cameras
- Add red-light cameras or toll booths

### Additional Features to Implement
- SD card storage for trip logs
- OLED display for visual alerts
- LED indicators for alert levels
- Bluetooth connectivity for smartphone app
- Over-the-air (OTA) updates for camera database

### Hardware Upgrades
- Add external antenna for better GPS reception
- Include battery backup for continuous operation
- Add temperature sensor for environmental monitoring
- Include accelerometer for harsh braking detection

## Safety Notice ⚠️

- **Primary Attention**: Always keep eyes on the road - do not rely solely on this device
- **Speed Limits**: Obey all traffic laws and posted speed limits
- **Device Placement**: Mount device securely to avoid driver distraction
- **Legal Compliance**: Check local laws regarding GPS detector devices
- **No Warranty**: This is an educational project - use at your own risk

## Contributing 🤝

Contributions are welcome! Please feel free to submit pull requests or open issues for:
- Bug fixes
- New features
- Documentation improvements
- Speed camera database updates for different regions

## License 📄

This project is open-source and available for educational purposes.

## Acknowledgments 🙏

- **TinyGPS++** library by Mikal Hart
- **ESP32** Arduino core by Espressif Systems
- **u-blox** for GPS module documentation

## Support 💬

For questions, issues, or suggestions:
- Open an issue on GitHub
- Check existing issues for solutions

---

**Remember**: This device is a safety aid, not a replacement for responsible driving. Always drive safely and obey traffic laws! 🚦
