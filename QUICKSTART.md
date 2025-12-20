# Quick Start Example

This guide will help you get SpeedZone-Alert up and running in 15 minutes.

## What You'll Need

**Hardware:**
- ESP32 board
- NEO-6M GPS module
- Active buzzer
- USB cable
- Breadboard and jumper wires

**Software:**
- Arduino IDE (installed)
- ESP32 board support (installed)
- TinyGPSPlus library (installed)

## 5-Minute Hardware Setup

### Step 1: Wire the GPS Module
```
GPS NEO-6M  →  ESP32
--------------------------
VCC         →  3.3V
GND         →  GND
TX          →  GPIO16 (RX)
RX          →  GPIO17 (TX)
```

### Step 2: Wire the Buzzer
```
Active Buzzer  →  ESP32
--------------------------
+/VCC          →  GPIO5
-/GND          →  GND
```

### Step 3: Connect USB
- Plug ESP32 into computer via USB cable
- ESP32 power LED should light up

## 5-Minute Software Setup

### Step 1: Open Arduino IDE
- Launch Arduino IDE
- Open `SpeedZone_Alert.ino`

### Step 2: Configure Board
- Go to: Tools → Board → ESP32 Dev Module
- Go to: Tools → Port → [Select your ESP32 port]

### Step 3: Add Your Speed Cameras
Edit the camera array in `SpeedZone_Alert.ino`:

```cpp
SpeedCamera cameras[] = {
  // Replace with your actual camera locations
  {"Your Camera 1", 40.7128, -74.0060, 50},
  {"Your Camera 2", 40.7589, -73.9851, 80},
};
```

**How to find coordinates:**
1. Go to Google Maps
2. Right-click on camera location
3. Click coordinates to copy
4. Paste into code (format: latitude, longitude)

### Step 4: Upload
- Click the Upload button (→)
- Wait for "Done uploading" message

## 5-Minute Testing

### Step 1: Open Serial Monitor
- Tools → Serial Monitor
- Set baud rate to: **115200**

### Step 2: Watch for Startup
You should see:
```
=== SpeedZone Alert System ===
Initializing...
System Ready!
Monitoring 2 speed camera locations
Waiting for GPS fix...
```

**Hear this:** 3 quick beeps from buzzer

### Step 3: Get GPS Fix
- Take device outdoors or near window
- GPS needs clear view of sky
- Wait 30-60 seconds for first fix

### Step 4: See GPS Data
Once GPS locks, you'll see:
```
--- Status Update ---
Position: 40.712345, -74.006789
Speed: 0.0 km/h
Nearest Camera: Your Camera 1
Distance: 1234 m
Speed Limit: 50 km/h
```

### Step 5: Test Alerts

**If within 500m of a camera:**
- See: `** Warning - Speed Camera Ahead **`
- Hear: Beep every 2 seconds

**If within 200m of a camera:**
- See: `*** DANGER ZONE - SLOW DOWN! ***`
- Hear: Rapid beeping every 0.5 seconds

**If speeding in danger zone:**
- See: `!!! SPEED LIMIT EXCEEDED !!!`
- Continues rapid beeping

## Troubleshooting

### Problem: "No GPS data received"
**Solution:**
1. Check GPS wiring (especially TX/RX crossed)
2. Move outdoors for better signal
3. Wait up to 60 seconds for first fix

### Problem: No buzzer sound
**Solution:**
1. Verify it's an ACTIVE buzzer (not passive)
2. Check buzzer polarity (+ to GPIO5, - to GND)
3. Test buzzer: connect directly to 3.3V temporarily

### Problem: No serial output
**Solution:**
1. Check baud rate is 115200
2. Verify correct COM port selected
3. Try pressing ESP32 reset button

### Problem: Won't upload
**Solution:**
1. Check USB cable (must support data, not charge-only)
2. Install CP210x or CH340 drivers if needed
3. Try different USB port
4. Press and hold BOOT button during upload

## Example Test Scenarios

### Scenario 1: Stationary Test
- Add a camera at your current location
- GPS coordinates from phone or Google Maps
- Should trigger danger zone alert immediately
- Tests: GPS, distance calculation, buzzer

### Scenario 2: Walking Test
- Add camera 300m away
- Walk toward it
- Should see warning at 500m, danger at 200m
- Tests: Real-time tracking, alert transitions

### Scenario 3: Driving Test (Passenger Only!)
- Add real camera locations
- Drive route past cameras
- Verify alerts at correct distances
- Tests: Speed detection, moving vehicle tracking

## Customization Examples

### Change Alert Distances
Edit `config.h`:
```cpp
#define WARNING_DISTANCE_M 1000  // Warning at 1km
#define DANGER_DISTANCE_M 300    // Danger at 300m
```

### Change Beep Speed
Edit `config.h`:
```cpp
#define WARNING_BEEP_INTERVAL 3000  // Slower warning
#define DANGER_BEEP_INTERVAL 300    // Faster danger
```

### Add More Cameras
Just add lines to the array:
```cpp
SpeedCamera cameras[] = {
  {"Camera 1", 40.7128, -74.0060, 50},
  {"Camera 2", 40.7589, -73.9851, 80},
  {"Camera 3", 40.7614, -73.9776, 60},  // Added
  {"Camera 4", 40.7489, -73.9680, 40},  // Added
};
```

### Change GPIO Pins
Edit `config.h`:
```cpp
#define GPS_RX_PIN 16    // Change if needed
#define GPS_TX_PIN 17    // Change if needed
#define BUZZER_PIN 5     // Change if needed
```

## Real-World Usage Tips

### For Daily Commute
1. Map your regular route
2. Add all cameras on route
3. Mount device on dashboard
4. Use car USB power

### For Road Trips
1. Research cameras on route ahead of time
2. Use regional databases (see CAMERA_DATABASE.md)
3. Bring power bank for longer trips
4. Update database for new regions

### For Testing/Development
1. Use test locations near home
2. Start with 2-3 cameras
3. Test walking before driving
4. Verify accuracy before road use

## Next Steps

### Enhance Your System
- [ ] Add OLED display for visual alerts
- [ ] Include SD card for trip logging
- [ ] Add LED indicators
- [ ] Implement Bluetooth for smartphone app
- [ ] Create permanent vehicle mount

### Expand Database
- [ ] Add cameras from local sources
- [ ] Create regional database
- [ ] Share with community
- [ ] Set up update routine

### Advanced Features
- [ ] Time-based speed limits (school zones)
- [ ] Direction-aware cameras
- [ ] Internet connectivity for live updates
- [ ] Multi-vehicle fleet tracking

## Common Use Cases

### Daily Driver
**Setup:**
- Permanent dashboard mount
- Powered by car USB
- Full local database
- Volume appropriate for vehicle

**Benefits:**
- Automatic alerts on commute
- Avoid speeding tickets
- Build safe driving habits

### Motorcycle
**Setup:**
- Waterproof enclosure
- Battery powered
- Helmet speaker or LED alerts
- Simplified controls

**Considerations:**
- Vibration resistance
- Weather protection
- Easy visibility

### Fleet Vehicle
**Setup:**
- Professional installation
- Integration with vehicle systems
- Logging capabilities
- Remote monitoring

**Benefits:**
- Driver safety training
- Reduced violations
- Insurance benefits

## Safety Reminders

✓ **DO:**
- Keep eyes on road
- Obey all speed limits
- Use as supplementary aid
- Drive defensively

✗ **DON'T:**
- Rely solely on device
- Operate while driving
- Ignore road signs
- Exceed speed limits

## Success Checklist

- [ ] Hardware properly wired
- [ ] GPS getting fix (< 60 seconds)
- [ ] Buzzer working (3 startup beeps)
- [ ] Serial output showing GPS data
- [ ] Cameras loaded (correct count shown)
- [ ] Distance calculations accurate
- [ ] Alerts triggering at correct distances
- [ ] Speed detection working
- [ ] Device securely mounted (for vehicle use)

## Getting Help

**If stuck:**
1. Check HARDWARE_GUIDE.md for detailed wiring
2. Review README.md for full documentation
3. See CAMERA_DATABASE.md for database help
4. Open GitHub issue with:
   - Problem description
   - Serial monitor output
   - Photos of wiring

**Common Questions:**
- GPS not working? → Check TX/RX crossing
- Buzzer silent? → Verify ACTIVE buzzer type
- No alerts? → Confirm camera coordinates
- Inaccurate distance? → Verify GPS accuracy

## Example Serial Output

**Successful Operation:**
```
=== SpeedZone Alert System ===
Initializing...
System Ready!
Monitoring 5 speed camera locations
Waiting for GPS fix...

--- Status Update ---
Position: 40.712800, -74.006000
Speed: 0.0 km/h
Nearest Camera: Camera 1 - Main Street
Distance: 450 m
Speed Limit: 50 km/h
** Warning - Speed Camera Ahead **

--- Status Update ---
Position: 40.713200, -74.005800
Speed: 45.5 km/h
Nearest Camera: Camera 1 - Main Street
Distance: 180 m
Speed Limit: 50 km/h
*** DANGER ZONE - SLOW DOWN! ***

--- Status Update ---
Position: 40.713500, -74.005600
Speed: 62.3 km/h
Nearest Camera: Camera 1 - Main Street
Distance: 120 m
Speed Limit: 50 km/h
*** DANGER ZONE - SLOW DOWN! ***

!!! SPEED LIMIT EXCEEDED !!!
Camera: Camera 1 - Main Street
Your Speed: 62.3 km/h | Limit: 50 km/h
Distance: 120 m
```

---

**Estimated Time to Complete:** 15 minutes
**Difficulty Level:** Beginner-Friendly
**Support:** GitHub Issues

Happy safe driving! 🚗💨
