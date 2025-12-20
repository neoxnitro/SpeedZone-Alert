# Hardware Setup Guide

## Detailed Component Specifications

### ESP32 Development Board
- **Voltage**: 3.3V logic level
- **Power**: Can be powered via USB (5V) or VIN pin (5-12V)
- **GPIO Pins**: Multiple available for sensors and outputs
- **Communication**: Multiple UART, I2C, SPI interfaces
- **Recommended**: ESP32 DevKit V1 or similar

### u-blox NEO-6M GPS Module
- **Chip**: u-blox NEO-6M
- **Channels**: 50 channels
- **Update Rate**: 1Hz (default), up to 5Hz
- **Accuracy**: 2.5m CEP
- **Protocol**: NMEA 0183
- **Baud Rate**: 9600 (default)
- **Voltage**: 3.3V (some modules have 5V regulator)
- **Current**: ~45mA during acquisition, ~35mA tracking
- **Cold Start**: ~27s
- **Hot Start**: ~1s

### Active Buzzer
- **Type**: Active (has internal oscillator)
- **Voltage**: 3.3V or 5V (check your module)
- **Sound**: Continuous tone when powered
- **Current**: ~20-30mA typical
- **Frequency**: ~2-4kHz typical
- **Note**: Passive buzzers require PWM signal and won't work with this code

## Complete Wiring Diagram

```
                    ┌─────────────┐
                    │   ESP32     │
                    │  DevKit V1  │
                    └─────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
   GPS NEO-6M        Active Buzzer      USB Power
   ┌─────────┐       ┌─────────┐       
   │ VCC─3.3V│◄──────┤3.3V     │       
   │ GND─GND │◄──────┤GND      │       
   │ TX─RX16 │◄──────┤GPIO16   │       
   │ RX─TX17 │◄──────┤GPIO17   │       
   │ [ANT]   │       └─────────┘       
   └─────────┘       ┌─────────┐
                     │ +  ─GPIO5│◄─────
                     │ -  ─GND  │◄─────
                     └─────────┘
```

## Step-by-Step Wiring Instructions

### 1. Prepare Your Workspace
- Use an anti-static mat or ground yourself
- Organize components and wires
- Have a multimeter ready for testing

### 2. Wire the GPS Module

**Connection Table:**
| GPS NEO-6M | ESP32 Pin | Wire Color (Suggested) |
|------------|-----------|------------------------|
| VCC        | 3.3V      | Red                    |
| GND        | GND       | Black                  |
| TX         | GPIO16    | Yellow/Green           |
| RX         | GPIO17    | Blue/White             |

**Important Notes:**
- **TX to RX**: GPS TX connects to ESP32 RX (GPIO16)
- **RX to TX**: GPS RX connects to ESP32 TX (GPIO17)
- **Power**: Most NEO-6M modules need 3.3V, but some have regulators for 5V
- **Antenna**: Ensure ceramic antenna faces upward

### 3. Wire the Active Buzzer

**Connection Table:**
| Buzzer | ESP32 Pin | Notes                    |
|--------|-----------|--------------------------|
| +/VCC  | GPIO5     | Signal pin               |
| -/GND  | GND       | Ground                   |

**Polarity Check:**
- Active buzzers have polarity (+ and -)
- Positive (+) pin is usually longer
- Some buzzers have a sticker on the back indicating polarity

### 4. Connect Power

**For Development/Testing:**
- Connect ESP32 to computer via USB cable
- USB provides 5V which ESP32 regulates to 3.3V

**For Vehicle Use:**
- Use car USB adapter (5V, min 1A)
- Or use 12V→5V DC-DC converter
- Add capacitors for power stability

## Breadboard Layout Example

```
           USB
            │
    ┌───────┴───────┐
    │     ESP32     │
    │               │
    │  3V3      GND │
    │   │        │  │
    │   │        │  │
    └───┼────────┼──┘
        │        │
    ┌───┼────────┼───┐
    │   │        │   │  Breadboard
    │  [+]      [-]  │
    │   │        │   │
    │   │ ┌──────┴───┼─ GPS GND
    │   │ │      │   │
    │   │ │  ┌───┴───┼─ Buzzer GND
    │   │ │  │       │
    │   └─┼──┼─ GPS VCC
    │     │  │       │
    │  GPIO5─┴─ Buzzer +
    │     │          │
    │ GPIO16─ GPS TX │
    │     │          │
    │ GPIO17─ GPS RX │
    │              │
    └──────────────┘
```

## Testing Your Connections

### 1. Visual Inspection
- Check all connections are secure
- Verify no shorts between pins
- Ensure correct orientation of components

### 2. Continuity Test
Use a multimeter in continuity mode:
- Test GND connections (should beep)
- Test 3.3V connections (should beep)
- Verify no shorts between signal and ground (should NOT beep)

### 3. Power Test
- Connect USB power
- ESP32 LED should light up
- GPS module LED should start blinking (may be slow at first)
- Buzzer should make 3 quick beeps (startup sequence)

### 4. GPS Signal Test
- Place device near window or outdoors
- Wait 30-60 seconds for GPS fix
- GPS LED should blink faster when locked
- Serial monitor should show GPS data

## Common Wiring Issues

### GPS Not Working
**Symptom**: "No GPS data received" message
**Possible Causes**:
1. RX/TX not crossed (TX→RX, RX→TX)
2. Wrong baud rate
3. Insufficient power
4. Damaged GPS module

**Solutions**:
- Swap RX and TX connections
- Verify GPS baud rate is 9600
- Check voltage at GPS VCC pin (should be 3.3V)
- Test GPS with different sketch

### Buzzer Not Working
**Symptom**: No sound from buzzer
**Possible Causes**:
1. Wrong buzzer type (passive vs active)
2. Reversed polarity
3. Insufficient power
4. Wrong GPIO pin

**Solutions**:
- Verify you have an ACTIVE buzzer (not passive)
- Reverse buzzer connections
- Test buzzer directly with 3.3V
- Check GPIO5 configuration in code

### ESP32 Not Powering On
**Symptom**: No LEDs, no response
**Possible Causes**:
1. Insufficient power supply
2. Short circuit
3. Damaged board

**Solutions**:
- Try different USB cable/port
- Check for shorts with multimeter
- Disconnect all peripherals and test ESP32 alone

## Enclosure Recommendations

### For Development
- Open breadboard setup is fine
- Keep GPS antenna clear
- Ensure buzzer can be heard

### For Vehicle Use
Consider:
- **Weather-proof enclosure** (if mounted externally)
- **Ventilation** for heat dissipation
- **GPS antenna placement**: Must face sky
- **Buzzer placement**: Must be audible
- **Secure mounting**: Use velcro, suction cup, or dashboard mount
- **Cable management**: Keep wires organized and secure

### Suggested Enclosures
- Small project boxes (80x50x30mm or larger)
- 3D printed custom case
- Repurposed electronics enclosure

## Power Supply Options

### Option 1: USB Power (Recommended for Beginners)
- **Source**: Computer or car USB adapter
- **Voltage**: 5V
- **Current**: 500mA minimum
- **Pros**: Simple, safe, widely available
- **Cons**: Requires USB connection

### Option 2: Vehicle Cigarette Lighter
- **Source**: 12V car outlet with USB adapter
- **Requirements**: 5V 1A USB adapter
- **Pros**: Convenient for vehicle use
- **Cons**: Drains car battery when engine off

### Option 3: Battery Pack
- **Source**: Portable power bank or battery
- **Voltage**: 5V via USB or 3.7V LiPo with regulator
- **Pros**: Portable, independent
- **Cons**: Requires recharging

### Option 4: Direct 12V Connection
- **Source**: Vehicle 12V supply
- **Requirements**: 12V→5V DC-DC buck converter (3A minimum)
- **Pros**: Permanent installation
- **Cons**: Requires proper wiring and protection

## Safety Considerations

### Electrical Safety
- Never exceed voltage ratings
- Use appropriate fuses for vehicle installations
- Avoid shorts by insulating exposed connections
- Double-check polarity before powering on

### Vehicle Installation Safety
- Do not obstruct driver's view
- Secure all components to prevent movement
- Route cables away from pedals and moving parts
- Do not interfere with airbags
- Consider professional installation for permanent setups

### General Safety
- Do not operate while driving
- Keep attention on the road
- Use device as supplementary aid only
- Comply with local laws regarding detection devices

## Maintenance

### Regular Checks
- Inspect connections monthly
- Clean GPS antenna surface
- Check for loose wires
- Verify buzzer still functional
- Update camera database as needed

### GPS Maintenance
- Keep antenna clean and unobstructed
- Avoid magnetic interference
- Periodically test GPS accuracy
- Update module firmware if available (advanced)

### Troubleshooting Tools
- Multimeter for voltage/continuity
- Serial monitor for diagnostics
- Spare components for testing
- Camera for documenting setup

## Upgrading Your Setup

### Optional Additions
1. **OLED Display** (I2C): Visual feedback
2. **SD Card Module** (SPI): Trip logging
3. **LED Indicators**: Status lights
4. **Voltage Monitor**: Battery level display
5. **Temperature Sensor**: Environmental data
6. **Bluetooth Module**: Smartphone connectivity

### Pin Availability
The following ESP32 pins are still available for expansion:
- GPIO2, GPIO4, GPIO13, GPIO14, GPIO15, GPIO18-23, GPIO25-27, GPIO32-33

## Resources

### Datasheets
- ESP32: https://www.espressif.com/en/products/socs/esp32
- NEO-6M: https://www.u-blox.com/en/product/neo-6-series
- TinyGPS++: http://arduiniana.org/libraries/tinygpsplus/

### Tools
- Fritzing: For creating wiring diagrams
- Eagle/KiCAD: For PCB design (advanced)
- Arduino IDE: For programming

### Suppliers
- GPS Modules: Amazon, eBay, AliExpress, Adafruit, SparkFun
- ESP32 Boards: Same as above
- Electronic components: DigiKey, Mouser, Arrow

---

For questions about hardware setup, please open an issue on GitHub.
