# Troubleshooting Guide

Complete troubleshooting reference for SpeedZone-Alert system.

## Table of Contents
1. [GPS Issues](#gps-issues)
2. [Buzzer Issues](#buzzer-issues)
3. [ESP32 Issues](#esp32-issues)
4. [Alert Issues](#alert-issues)
5. [Power Issues](#power-issues)
6. [Software Issues](#software-issues)
7. [Accuracy Issues](#accuracy-issues)

---

## GPS Issues

### No GPS Data Received

**Symptom:**
```
No GPS data received. Check wiring!
```

**Causes & Solutions:**

1. **Incorrect TX/RX Wiring**
   - GPS TX must connect to ESP32 RX (GPIO16)
   - GPS RX must connect to ESP32 TX (GPIO17)
   - Solution: Swap TX and RX connections

2. **No Power to GPS**
   - Check: GPS VCC connected to ESP32 3.3V
   - Check: GPS GND connected to ESP32 GND
   - Test: GPS LED should blink (slowly when searching, faster when locked)

3. **Wrong Baud Rate**
   - Default NEO-6M baud rate: 9600
   - Check config.h: `#define GPS_BAUD 9600`
   - If changed GPS baud, update config.h

4. **Damaged GPS Module**
   - Test with separate GPS test sketch
   - Try different GPS module

**Diagnostic Steps:**
```cpp
// Add to loop() for debugging:
Serial.print("GPS chars processed: ");
Serial.println(gps.charsProcessed());
Serial.print("GPS sentences failed: ");
Serial.println(gps.failedChecksum());
```

### GPS Not Getting Fix

**Symptom:**
- GPS receiving data but no valid location
- No status updates appearing

**Causes & Solutions:**

1. **Insufficient Sky View**
   - GPS needs direct view of sky
   - Solution: Move outdoors or near window
   - Avoid: Inside buildings, under trees, near tall buildings

2. **Cold Start Delay**
   - First fix can take 30-60 seconds
   - Subsequent fixes: 1-5 seconds (hot start)
   - Solution: Be patient on first startup

3. **Weak Antenna**
   - Check antenna connection to GPS module
   - Ensure antenna faces upward
   - Consider external active antenna

4. **Interference**
   - Keep away from strong magnetic fields
   - Avoid close proximity to WiFi routers
   - Move away from other RF devices

**Check GPS Status:**
```cpp
// Add to processGPSData():
Serial.print("Satellites: ");
Serial.println(gps.satellites.value());
Serial.print("HDOP: ");
Serial.println(gps.hdop.hdop());
```
- Need: 4+ satellites for fix
- Good HDOP: < 2.0

### GPS Data Erratic

**Symptom:**
- Jumping coordinates
- Inaccurate positions
- Speed fluctuations

**Causes & Solutions:**

1. **Poor Signal Quality**
   - Check satellite count (need 6+ for accuracy)
   - Improve antenna placement
   - Reduce obstructions

2. **Multipath Interference**
   - GPS signals reflecting off buildings
   - Solution: Move away from tall structures
   - Use in open areas when possible

3. **EMI (Electromagnetic Interference)**
   - ESP32 WiFi can interfere
   - Solution: Disable WiFi if not needed:
   ```cpp
   WiFi.mode(WIFI_OFF);
   ```

---

## Buzzer Issues

### No Sound from Buzzer

**Symptom:**
- No startup beeps
- No alert beeps
- Serial output shows alerts but no sound

**Causes & Solutions:**

1. **Wrong Buzzer Type**
   - Must be ACTIVE buzzer (has internal oscillator)
   - PASSIVE buzzers need PWM (won't work with simple HIGH/LOW)
   - Test: Connect buzzer directly to 3.3V - active buzzer will sound

2. **Reversed Polarity**
   - Buzzers have + and - pins
   - Positive (+) to GPIO5
   - Negative (-) to GND
   - Solution: Swap connections

3. **Insufficient Power**
   - Check buzzer voltage rating
   - Some 5V buzzers are quiet at 3.3V
   - Solution: Use 3.3V rated buzzer or level shifter

4. **Wrong GPIO Pin**
   - Verify GPIO5 in wiring
   - Check config.h matches physical wiring
   - Some ESP32 pins are input-only (won't work)

**Test Code:**
```cpp
// Add to setup() for testing:
digitalWrite(BUZZER_PIN, HIGH);
delay(1000);
digitalWrite(BUZZER_PIN, LOW);
```

### Buzzer Stays On

**Symptom:**
- Continuous sound
- Won't stop beeping

**Causes & Solutions:**

1. **Code Stuck in Loop**
   - Check for infinite loops
   - Verify proper alert state management

2. **Hardware Short**
   - Check for short circuit
   - Buzzer pin to VCC

**Quick Fix:**
- Upload blank sketch to stop
- Check wiring before re-uploading

### Buzzer Too Quiet/Loud

**Solutions:**

1. **Volume Control:**
   - Add resistor in series (1K-10K) to reduce volume
   - Different buzzer models have different volumes

2. **Change Beep Duration:**
   Edit config.h:
   ```cpp
   #define BEEP_DURATION 50  // Shorter = quieter
   ```

---

## ESP32 Issues

### Won't Upload Code

**Symptom:**
- "Failed to connect" error
- "Timed out waiting for packet header"

**Causes & Solutions:**

1. **Wrong Port Selected**
   - Check: Tools → Port
   - Try all available ports
   - Unplug/replug ESP32

2. **Driver Issues**
   - Check device manager for unknown devices
   - Install CP210x or CH340 drivers
   - Windows: Verify driver in Device Manager
   - Mac/Linux: Check with `ls /dev/tty*`

3. **Upload Mode**
   - Hold BOOT button while clicking Upload
   - Release BOOT after "Connecting..." appears
   - Some boards auto-enter bootloader mode

4. **Bad USB Cable**
   - Use data cable (not charge-only)
   - Try different cable
   - Try different USB port

5. **Wrong Board Selected**
   - Verify: Tools → Board → ESP32 Dev Module
   - Try: "DOIT ESP32 DEVKIT V1" if Dev Module doesn't work

### ESP32 Keeps Resetting

**Symptom:**
- Continuous boot loops
- Random resets
- "Brownout detector" messages

**Causes & Solutions:**

1. **Insufficient Power**
   - USB port not providing enough current
   - Solution: Use powered USB hub
   - Solution: Use external 5V supply to VIN pin

2. **Power Surges**
   - GPS or buzzer drawing too much current
   - Add capacitor: 100µF across ESP32 power pins

3. **Loose Connections**
   - Check all connections secure
   - Use breadboard with good contacts

### ESP32 Overheating

**Symptom:**
- Board hot to touch
- Unexpected resets

**Solutions:**
1. Check for short circuits
2. Reduce buzzer beep duration
3. Add heatsink to ESP32
4. Ensure proper ventilation

---

## Alert Issues

### No Alerts Near Cameras

**Symptom:**
- GPS working
- Near known camera
- No warning or danger alerts

**Causes & Solutions:**

1. **Wrong Camera Coordinates**
   - Verify latitude/longitude correct
   - Check coordinate format (decimal degrees)
   - Use Google Maps to verify
   - Test: Add camera at current GPS position

2. **Alert Distance Too Small**
   - Check config.h settings
   - Increase WARNING_DISTANCE_M:
   ```cpp
   #define WARNING_DISTANCE_M 1000  // Test with 1km
   ```

3. **Distance Calculation Error**
   - Add debug output:
   ```cpp
   Serial.print("Camera: ");
   Serial.print(cameras[i].name);
   Serial.print(" Distance: ");
   Serial.println(distance);
   ```

### Alerts Too Frequent

**Symptom:**
- Alert when far from cameras
- Constant beeping

**Causes & Solutions:**

1. **Alert Distance Too Large**
   - Reduce WARNING_DISTANCE_M
   - Reduce DANGER_DISTANCE_M

2. **Incorrect Camera Location**
   - Verify camera coordinates
   - Remove test cameras

3. **GPS Drift**
   - Check GPS accuracy
   - Move to better location for signal

### Speed Detection Not Working

**Symptom:**
- Speed shown as 0.0 km/h when moving
- No speed warnings when exceeding limit

**Causes & Solutions:**

1. **Not Moving Fast Enough**
   - GPS speed accuracy: ±0.1 km/h
   - Need to be moving for speed reading
   - Walking speed may show 0 or erratic

2. **Poor GPS Fix**
   - Need good satellite lock for speed
   - Check satellite count
   - Improve signal reception

3. **Speed Calculation Issue**
   - TinyGPS++ provides speed in kmph:
   ```cpp
   double speed = gps.speed.kmph();
   ```

---

## Power Issues

### Device Not Powering On

**Causes & Solutions:**

1. **No Power Connection**
   - Check USB cable plugged in
   - Check ESP32 USB port
   - Try different power source

2. **Damaged Board**
   - Check for burn marks
   - Test with multimeter: 5V at VIN, 3.3V at 3V3 pin

### Battery Drains Quickly

**Causes:**
- GPS module: 35-45mA
- ESP32: 80-240mA (depending on WiFi/Bluetooth)
- Buzzer: 20-30mA when active

**Solutions:**
1. Disable WiFi/Bluetooth:
   ```cpp
   WiFi.mode(WIFI_OFF);
   btStop();
   ```

2. Reduce GPS update rate (advanced)

3. Sleep mode between alerts (advanced)

4. Larger battery capacity

---

## Software Issues

### Compilation Errors

**Error: "TinyGPS++.h: No such file"**
- Solution: Install TinyGPSPlus library
- Arduino IDE → Sketch → Include Library → Manage Libraries
- Search "TinyGPSPlus" → Install

**Error: "HardwareSerial.h: No such file"**
- Solution: Install ESP32 board support
- Check board manager URL added
- Reinstall ESP32 platform

**Error: "config.h: No such file"**
- Solution: config.h must be in same folder as .ino file
- Verify file exists
- Check filename case-sensitive on Linux/Mac

### Memory Issues

**Error: "Not enough memory"**
- Too many cameras in database
- Solution: Use PROGMEM:
```cpp
const PROGMEM SpeedCamera cameras[] = { ... };
```

**Error: "Stack overflow"**
- Increase stack size (advanced)
- Reduce local variables
- Optimize memory usage

### Strange Behavior

**Random Characters in Serial**
- Wrong baud rate: Set to 115200
- EMI: Add capacitors, improve wiring

**Crashes/Freezes**
- Add watchdog timer (advanced)
- Check for infinite loops
- Verify all pointers valid

---

## Accuracy Issues

### Distance Calculation Inaccurate

**Symptom:**
- Distance shown doesn't match reality
- Alerts at wrong distances

**Solutions:**

1. **Verify Haversine Implementation**
   - Current implementation is correct for < 10km
   - Very accurate for typical use case

2. **Check Coordinate Format**
   - Must be decimal degrees
   - Latitude: -90 to +90
   - Longitude: -180 to +180

3. **GPS Accuracy**
   - Consumer GPS: ±5-10m typical
   - Distance calculations accurate to ±1m with good fix

### Speed Reading Inaccurate

**Causes:**
- GPS speed calculation delay
- Poor satellite fix
- Multipath interference

**Solutions:**
1. Need 6+ satellites for accurate speed
2. Open area with clear sky view
3. Speed more accurate when moving straight

---

## Diagnostic Tools

### Enable Debug Output

Add to code for detailed debugging:

```cpp
// At top of file:
#define DEBUG 1

// In loop():
#if DEBUG
  Serial.print("Satellites: ");
  Serial.print(gps.satellites.value());
  Serial.print(" HDOP: ");
  Serial.print(gps.hdop.hdop());
  Serial.print(" Speed: ");
  Serial.println(gps.speed.kmph());
#endif
```

### Test Individual Components

**GPS Only:**
```cpp
void loop() {
  while (gpsSerial.available()) {
    Serial.write(gpsSerial.read());
  }
}
```

**Buzzer Only:**
```cpp
void loop() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);
}
```

### Check GPS NMEA Sentences

Add to loop():
```cpp
while (gpsSerial.available()) {
  char c = gpsSerial.read();
  Serial.write(c);  // Raw GPS data
  gps.encode(c);
}
```

Look for: $GPGGA, $GPRMC sentences

---

## Getting Help

If problem persists:

1. **Collect Information:**
   - Complete serial monitor output
   - Photos of wiring
   - ESP32 board model
   - GPS module model
   - Exact error messages

2. **Check Documentation:**
   - README.md
   - HARDWARE_GUIDE.md
   - This file

3. **Search Issues:**
   - GitHub Issues page
   - Similar problems may be solved

4. **Open New Issue:**
   - Provide all collected information
   - Describe steps to reproduce
   - Include attempted solutions

## Common Error Messages

| Error | Meaning | Solution |
|-------|---------|----------|
| "No GPS data received" | No serial data from GPS | Check wiring, especially TX/RX |
| "charsProcessed < 10" | GPS not sending data | Check power, baud rate, wiring |
| "Brownout detector" | Voltage drop | Better power supply, add capacitor |
| "Core panic" | Software crash | Check for bugs, verify memory usage |
| "Failed checksum" | Corrupted GPS data | Check wiring, EMI, try lower baud |

---

**Still having issues?** Open a GitHub issue with full details!
