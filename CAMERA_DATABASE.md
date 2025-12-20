# Speed Camera Database Template

This document provides guidance on how to populate and maintain your speed camera database.

## Database Structure

Each speed camera entry in the database contains:
- **name**: Descriptive name/location identifier
- **latitude**: GPS latitude in decimal degrees
- **longitude**: GPS longitude in decimal degrees  
- **speedLimit**: Speed limit at that location in km/h

## How to Add Camera Locations

### Step 1: Find GPS Coordinates

**Using Google Maps:**
1. Navigate to Google Maps (https://maps.google.com)
2. Find the speed camera location
3. Right-click on the exact location
4. Click the coordinates at the top to copy them
5. Format: `latitude, longitude` (e.g., 40.7128, -74.0060)

**Using GPS Device:**
1. Drive to the camera location
2. Note the coordinates from your GPS device
3. Convert to decimal degrees if needed

**Coordinate Formats:**
- **Decimal Degrees (DD)**: 40.7128, -74.0060 ✓ (Use this format)
- **Degrees Minutes Seconds (DMS)**: 40°42'46"N 74°00'22"W ✗ (Convert to DD)

**Conversion Tools:**
- Online: https://www.latlong.net/
- Many GPS apps provide decimal format

### Step 2: Determine Speed Limit
- Check posted speed limit signs
- Verify with local traffic regulations
- Consider different limits for different times (school zones, etc.)

### Step 3: Add to Database

Open `SpeedZone_Alert.ino` and locate the camera array:

```cpp
SpeedCamera cameras[] = {
  // Format: {"Name", latitude, longitude, speedLimit_km/h}
  {"Camera 1 - Main Street", 40.7128, -74.0060, 50},
  {"Camera 2 - Highway 101", 40.7589, -73.9851, 80},
  // Add your cameras below:
  
};
```

### Step 4: Test Your Entries
1. Upload code to ESP32
2. Open Serial Monitor
3. Verify all cameras are loaded: "Monitoring X speed camera locations"
4. Drive near a camera location to test alerts

## Example Entries

### Urban Speed Camera
```cpp
{"Downtown Main St", 40.7128, -74.0060, 50},
```
- **Location**: City center, main street
- **Speed Limit**: 50 km/h (31 mph)
- **Alert Distance**: Warning at 500m, Danger at 200m

### Highway Speed Camera
```cpp
{"I-95 Northbound Mile 45", 38.9072, -77.0369, 100},
```
- **Location**: Interstate highway
- **Speed Limit**: 100 km/h (62 mph)
- **Higher limit for highway speeds**

### School Zone Camera
```cpp
{"Elementary School Zone", 40.7589, -73.9851, 30},
```
- **Location**: School zone
- **Speed Limit**: 30 km/h (19 mph)
- **Lower limit for safety**

## Database Best Practices

### Naming Conventions
Use descriptive names that include:
- Street name or route number
- Direction (if applicable)
- Landmark or cross-street
- Type of zone (school, residential, highway)

**Good Examples:**
- `"Main St & Oak Ave - Southbound"`
- `"Highway 101 MM 45.2 Northbound"`
- `"School Zone - Lincoln Elementary"`
- `"Downtown Redlight - 5th & Broadway"`

**Avoid:**
- `"Camera1"` (not descriptive)
- `"Speedcam"` (unclear location)

### Coordinate Precision
- Use 6 decimal places for accuracy
- 6 decimals ≈ 0.1 meter precision
- More decimals = higher accuracy
- Example: `40.712800` not `40.71`

### Speed Limits
- Always use km/h (code uses metric)
- Convert mph if needed: `mph × 1.60934 = km/h`
- Examples:
  - 30 mph = 48 km/h (use 50)
  - 55 mph = 88 km/h (use 90)
  - 65 mph = 105 km/h (use 105)

### Organization Tips
- Group by region/city
- Sort by road/highway
- Add comments for special cases
- Keep backup of your database

## Sample Databases

### Urban City Center
```cpp
SpeedCamera cameras[] = {
  // Downtown District
  {"Main St & 1st Ave", 40.7128, -74.0060, 50},
  {"Broadway & 5th St", 40.7150, -74.0045, 50},
  {"Park Ave & Central", 40.7180, -74.0030, 40},
  
  // Residential Areas
  {"Oak Street School Zone", 40.7200, -74.0080, 30},
  {"Maple Ave Residential", 40.7220, -74.0100, 40},
  
  // Highway Entrances
  {"I-95 On-Ramp", 40.7250, -74.0120, 60},
};
```

### Highway Corridor
```cpp
SpeedCamera cameras[] = {
  // Highway 101 Northbound
  {"HWY 101 MM 42.1 NB", 37.7749, -122.4194, 105},
  {"HWY 101 MM 45.8 NB", 37.7850, -122.4180, 105},
  {"HWY 101 MM 48.3 NB", 37.7950, -122.4165, 105},
  
  // Highway 101 Southbound  
  {"HWY 101 MM 48.3 SB", 37.7950, -122.4170, 105},
  {"HWY 101 MM 45.8 SB", 37.7850, -122.4185, 105},
  {"HWY 101 MM 42.1 SB", 37.7749, -122.4199, 105},
};
```

### Mixed Area
```cpp
SpeedCamera cameras[] = {
  // Urban
  {"City Center - Main St", 40.7128, -74.0060, 50},
  {"School Zone - Elementary", 40.7150, -74.0080, 30},
  
  // Suburban
  {"Suburban Blvd", 40.7300, -74.0200, 60},
  {"Commercial District", 40.7350, -74.0250, 50},
  
  // Highway
  {"Highway Access Road", 40.7500, -74.0400, 80},
  {"Interstate I-95", 40.7600, -74.0500, 105},
};
```

## Database Limitations

### Current System
- **Storage**: Array in program memory (PROGMEM)
- **Capacity**: Limited by ESP32 RAM (typically ~100-200 cameras)
- **Updates**: Requires code recompilation and upload

### Large Database Solutions
If you need to store more cameras:

1. **Use PROGMEM** (store in flash memory):
```cpp
const PROGMEM SpeedCamera cameras[] = {
  // your cameras
};
```

2. **SD Card Storage** (advanced):
   - Store cameras in CSV file on SD card
   - Read at startup
   - Virtually unlimited capacity

3. **Split by Region**:
   - Create multiple versions for different regions
   - Load based on GPS area detection

## Updating Your Database

### When to Update
- New speed cameras installed
- Speed limits change
- Camera locations move
- Cameras removed

### Update Process
1. Edit camera array in `SpeedZone_Alert.ino`
2. Save file
3. Upload to ESP32
4. Test new entries

### Backup Strategy
- Keep a backup copy of your customized code
- Document your camera locations separately
- Consider version control (Git)

## Community Database

### Sharing Your Database
If you want to contribute your regional database:
1. Create a descriptive file (e.g., `cameras_london_uk.txt`)
2. Format entries clearly
3. Include region/country information
4. Submit as pull request or issue

### Using Community Databases
- Check GitHub issues for shared databases
- Verify accuracy before use
- Update coordinates if needed
- Report errors or outdated information

## Legal Considerations

### Important Notes
- **Local Laws**: Check if speed camera detectors are legal in your area
- **Accuracy**: Database is only as accurate as the data provided
- **Liability**: User is responsible for verifying information
- **Updates**: Speed cameras may be added, moved, or removed
- **Usage**: This is an educational project - use responsibly

### Restricted Regions
Some countries/regions prohibit speed camera detectors:
- Check local regulations
- Consult legal advice if unsure
- Do not use in prohibited areas

## Advanced Features

### Time-Based Speed Limits
For zones with variable speed limits (e.g., school zones):

```cpp
// This requires code modification to check time
struct SpeedCamera {
  const char* name;
  double latitude;
  double longitude;
  int speedLimit;
  int schoolZoneLimit;  // Add this field
  // Check time and use appropriate limit
};
```

### Direction-Specific Cameras
For cameras that only monitor one direction:

```cpp
// Add bearing/direction checking (advanced)
struct SpeedCamera {
  const char* name;
  double latitude;
  double longitude;
  int speedLimit;
  double bearing;  // Direction camera monitors (0-360°)
};
```

### Mobile Speed Cameras
Mobile/temporary cameras are harder to track:
- Require frequent database updates
- Consider crowd-sourced data
- May need internet connectivity

## Troubleshooting

### Camera Not Triggering
**Problem**: Driving past camera but no alert
**Solutions**:
- Verify coordinates are correct (within 10 meters)
- Check speed limit is appropriate
- Increase WARNING_DISTANCE_M in config.h
- Test distance calculation with known location

### Too Many False Alerts
**Problem**: Alerts when not near cameras
**Solutions**:
- Verify GPS accuracy (< 10m)
- Check for duplicate entries
- Reduce WARNING_DISTANCE_M in config.h
- Remove outdated/incorrect cameras

### Database Too Large
**Problem**: ESP32 running out of memory
**Solutions**:
- Remove unused cameras
- Use PROGMEM to store in flash
- Implement SD card storage
- Create regional databases

## Data Sources

### Finding Camera Locations
- **Official**: Local government/police websites
- **Community**: Crowd-sourced databases (verify accuracy)
- **Apps**: Waze, Google Maps (may have camera markers)
- **Personal**: Manual mapping during drives

### Accuracy Verification
- Cross-reference multiple sources
- Test with known cameras
- Update based on field testing
- Community feedback

## Template File

Create a text file with your cameras for easy backup:

```
# My Speed Camera Database
# Region: [Your City/Region]
# Last Updated: [Date]

Name, Latitude, Longitude, Speed Limit (km/h)
Main Street Camera, 40.7128, -74.0060, 50
Highway 101 MM45, 40.7589, -73.9851, 80
School Zone Oak St, 40.7614, -73.9776, 30
```

Then convert to code format when ready.

---

**Remember**: Always verify camera locations and speed limits for accuracy. Drive safely and obey all traffic laws! 🚦
