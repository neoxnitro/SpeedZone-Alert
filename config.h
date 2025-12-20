/*
 * Configuration file for SpeedZone Alert System
 * 
 * Modify these settings to match your hardware setup and preferences
 */

#ifndef CONFIG_H
#define CONFIG_H

// ===== GPS Configuration =====
#define GPS_BAUD 9600           // GPS module baud rate (standard for NEO-6M)
#define GPS_RX_PIN 16           // ESP32 pin connected to GPS TX
#define GPS_TX_PIN 17           // ESP32 pin connected to GPS RX

// ===== Buzzer Configuration =====
#define BUZZER_PIN 5            // ESP32 pin connected to active buzzer

// ===== Alert Distance Thresholds (in meters) =====
#define WARNING_DISTANCE_M 500  // Start warning at 500m from camera
#define DANGER_DISTANCE_M 200   // Danger zone at 200m from camera

// ===== Buzzer Alert Intervals (in milliseconds) =====
#define WARNING_BEEP_INTERVAL 2000  // Beep every 2 seconds in warning zone
#define DANGER_BEEP_INTERVAL 500    // Beep every 0.5 seconds in danger zone
#define BEEP_DURATION 100           // Duration of each beep in ms

// ===== Speed Detection =====
// Speed warnings are automatically triggered when current speed exceeds
// the speed limit of nearby cameras in the danger zone

#endif // CONFIG_H
