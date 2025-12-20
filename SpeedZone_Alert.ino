/*
 * SpeedZone Alert - ESP32 Road Safety Assistant
 * 
 * An ESP32-based system using GPS (u-blox NEO-6M) to detect proximity to speed cameras
 * and provide intelligent alerts via an active buzzer.
 * 
 * Features:
 * - Real-time GPS tracking
 * - Speed camera database with fixed locations
 * - Proximity-based alerts (warning and danger zones)
 * - Speed threshold detection
 * - Active buzzer alerts with different patterns
 */

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "config.h"

// GPS objects
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use UART1 for GPS

// Speed camera structure
struct SpeedCamera {
  const char* name;
  double latitude;
  double longitude;
  int speedLimit; // km/h
};

// Speed camera database - Add your local speed camera locations here
SpeedCamera cameras[] = {
  {"Camera 1 - Main Street", 40.7128, -74.0060, 50},
  {"Camera 2 - Highway 101", 40.7589, -73.9851, 80},
  {"Camera 3 - Park Avenue", 40.7614, -73.9776, 60},
  {"Camera 4 - Downtown", 40.7489, -73.9680, 40},
  {"Camera 5 - School Zone", 40.7549, -73.9840, 30}
};

const int numCameras = sizeof(cameras) / sizeof(cameras[0]);

// Alert state
enum AlertState {
  NONE,
  WARNING,
  DANGER
};

AlertState currentAlert = NONE;
int nearestCameraIndex = -1;
double nearestDistance = 999999.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== SpeedZone Alert System ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize GPS (RX=16, TX=17)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Startup beep
  startupBeep();
  
  Serial.println(F("System Ready!"));
  Serial.print(F("Monitoring "));
  Serial.print(numCameras);
  Serial.println(F(" speed camera locations"));
  Serial.println(F("Waiting for GPS fix...\n"));
}

void loop() {
  // Read GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      processGPSData();
    }
  }
  
  // Check if GPS data is stale
  static unsigned long lastGPSWarning = 0;
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    if (millis() - lastGPSWarning > 5000) {
      Serial.println(F("No GPS data received. Check wiring!"));
      lastGPSWarning = millis();
    }
  }
  
  // Handle buzzer alerts
  handleAlerts();
}

void processGPSData() {
  if (!gps.location.isValid()) {
    return;
  }
  
  double currentLat = gps.location.lat();
  double currentLon = gps.location.lng();
  double currentSpeed = gps.speed.kmph();
  
  // Find nearest camera and calculate distance
  nearestDistance = 999999.0;
  nearestCameraIndex = -1;
  AlertState newAlert = NONE;
  
  for (int i = 0; i < numCameras; i++) {
    double distance = calculateDistance(currentLat, currentLon, 
                                       cameras[i].latitude, cameras[i].longitude);
    
    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearestCameraIndex = i;
    }
    
    // Check proximity thresholds
    if (distance <= DANGER_DISTANCE_M) {
      newAlert = DANGER;
      // Check speed threshold
      if (currentSpeed > cameras[i].speedLimit) {
        // Over speed limit!
        displaySpeedWarning(i, currentSpeed, distance);
      }
    } else if (distance <= WARNING_DISTANCE_M) {
      if (newAlert != DANGER) {
        newAlert = WARNING;
      }
    }
  }
  
  // Update alert state
  if (currentAlert != newAlert) {
    currentAlert = newAlert;
    displayStatus(currentLat, currentLon, currentSpeed);
  }
  
  // Periodic status update
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 5000) {
    displayStatus(currentLat, currentLon, currentSpeed);
    lastUpdate = millis();
  }
}

void displayStatus(double lat, double lon, double speed) {
  Serial.println(F("\n--- Status Update ---"));
  Serial.print(F("Position: "));
  Serial.print(lat, 6);
  Serial.print(F(", "));
  Serial.println(lon, 6);
  Serial.print(F("Speed: "));
  Serial.print(speed, 1);
  Serial.println(F(" km/h"));
  
  if (nearestCameraIndex >= 0) {
    Serial.print(F("Nearest Camera: "));
    Serial.println(cameras[nearestCameraIndex].name);
    Serial.print(F("Distance: "));
    Serial.print(nearestDistance, 0);
    Serial.println(F(" m"));
    Serial.print(F("Speed Limit: "));
    Serial.print(cameras[nearestCameraIndex].speedLimit);
    Serial.println(F(" km/h"));
    
    if (currentAlert == DANGER) {
      Serial.println(F("*** DANGER ZONE - SLOW DOWN! ***"));
    } else if (currentAlert == WARNING) {
      Serial.println(F("** Warning - Speed Camera Ahead **"));
    }
  }
  Serial.println();
}

void displaySpeedWarning(int cameraIndex, double speed, double distance) {
  Serial.println(F("\n!!! SPEED LIMIT EXCEEDED !!!"));
  Serial.print(F("Camera: "));
  Serial.println(cameras[cameraIndex].name);
  Serial.print(F("Your Speed: "));
  Serial.print(speed, 1);
  Serial.print(F(" km/h | Limit: "));
  Serial.print(cameras[cameraIndex].speedLimit);
  Serial.println(F(" km/h"));
  Serial.print(F("Distance: "));
  Serial.print(distance, 0);
  Serial.println(F(" m"));
  Serial.println();
}

void handleAlerts() {
  static unsigned long lastBeepTime = 0;
  static unsigned long beepStartTime = 0;
  static bool isBeeping = false;
  unsigned long currentTime = millis();
  
  // Handle active beep completion
  if (isBeeping && (currentTime - beepStartTime >= BEEP_DURATION)) {
    digitalWrite(BUZZER_PIN, LOW);
    isBeeping = false;
  }
  
  // Check if it's time for a new beep
  if (!isBeeping) {
    bool shouldBeep = false;
    unsigned long beepInterval = 0;
    
    switch (currentAlert) {
      case DANGER:
        // Rapid beeping in danger zone
        beepInterval = DANGER_BEEP_INTERVAL;
        shouldBeep = (currentTime - lastBeepTime > beepInterval);
        break;
        
      case WARNING:
        // Slower beeping in warning zone
        beepInterval = WARNING_BEEP_INTERVAL;
        shouldBeep = (currentTime - lastBeepTime > beepInterval);
        break;
        
      case NONE:
      default:
        // No alert
        break;
    }
    
    if (shouldBeep) {
      digitalWrite(BUZZER_PIN, HIGH);
      beepStartTime = currentTime;
      lastBeepTime = currentTime;
      isBeeping = true;
    }
  }
}

// Haversine formula to calculate distance between two GPS coordinates
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Earth's radius in meters
  
  double phi1 = lat1 * PI / 180.0;
  double phi2 = lat2 * PI / 180.0;
  double deltaPhi = (lat2 - lat1) * PI / 180.0;
  double deltaLambda = (lon2 - lon1) * PI / 180.0;
  
  double a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) +
             cos(phi1) * cos(phi2) *
             sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  
  return R * c; // Distance in meters
}

void startupBeep() {
  // Three quick beeps to indicate system start
  // Note: Blocking delays are acceptable during setup
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}
