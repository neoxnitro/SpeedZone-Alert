#include <Arduino.h>

// Define TOUCH_CS if you don't need the touch controller or
// if your board doesn't expose a touch controller pin.
// Set to a valid pin number if you have a touch controller (e.g. 32).
#ifndef TOUCH_CS
#define TOUCH_CS -1 // -1 disables touch support in TFT_eSPI
#endif

#include <TFT_eSPI.h>

// --- Attempt to enable display backlight automatically ---
// If your board has a backlight control pin, define it here. The
// TTGO LoRa boards often expose BL on GPIO 32 but this may vary.

#include <WiFi.h>
#include <time.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>

TFT_eSPI tft = TFT_eSPI();

// WiFi credentials
const char *ssid1 = "Livebox6-1CB6-24G";
const char *pass1 = "dxGVxr6Kb6LV";
const char *ssid2 = "TP-Link_25FE";
const char *pass2 = "37522872";

// Set your trigger epoch here (seconds since 1970)
#define trigger_date 1763643003UL

// Watchdog timeout (seconds)
#define WDT_TIMEOUT 10

// Servo configuration
const int SERVO_PIN = 13;
Servo myservo;
bool servoTriggered = false;

void showMessage(const char *msg, int y, uint16_t color = TFT_WHITE, uint8_t size = 2)
{
  tft.fillRect(0, y, 240, 20 * size, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(size);
  tft.setCursor(5, y);
  tft.print(msg);
}

bool tryConnect(const char *ssid, const char *pass, const char *tryMsg)
{
  showMessage(tryMsg, 40, TFT_WHITE, 2);
  WiFi.begin(ssid, pass);
  unsigned long start = millis();
  while (millis() - start < 10000) // wait 10s for connection
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      showMessage("Wifi connected", 70, TFT_GREEN, 2);
      delay(3000);
      return true;
    }
    delay(500);
  }
  return false;
}

void _setup()
{
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
#endif

  // initial boot message
  showMessage("Booting ...", 10, TFT_WHITE, 2);
  delay(1000);

  // attach servo and set to 0° (not triggered)
  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_PIN);
  myservo.write(10);
  delay(1000);
  myservo.detach();

  // Try Wifi 1
  if (tryConnect(ssid1, pass1, "Try Wifi 1"))
  {
    configTzTime("Atlantic/Canary", "pool.ntp.org", "time.nist.gov");
  }
  else
  {
    // Try Wifi 2
    if (tryConnect(ssid2, pass2, "Try Wifi 2"))
    {
      configTzTime("Atlantic/Canary", "pool.ntp.org", "time.nist.gov");
    }
    else
    {
      showMessage("Wifi failed", 70, TFT_RED, 2);
      // No WiFi/time available; still continue to show boot time placeholder
    }
    // clear screen after attempts
    tft.fillScreen(TFT_BLACK);
  }

  // Initialize and enable watchdog for this task
  esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT, true);
  if (err == ESP_OK)
  {
    esp_task_wdt_add(NULL); // add current thread to WDT
    showMessage("Watchdog enabled", 100, TFT_WHITE, 2);
    delay(200);
  }
  else
  {
    showMessage("WDT init failed", 100, TFT_RED, 2);
    delay(1000);
  }
}

void _loop()
{
  time_t now = time(nullptr);
  struct tm timeinfo;
  char timestr[32];

  if (localtime_r(&now, &timeinfo))
  {
    // Format: DD/MM - HHhMM
    snprintf(timestr, sizeof(timestr), "%02d/%02d - %02dh%02d", timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_hour, timeinfo.tm_min);
  }
  else
  {
    strcpy(timestr, "No Time");
  }

  // calculate remaining minutes until trigger
  long remainingMinutes = (long)(((long long)trigger_date - (long long)now) / 60);
  if (remainingMinutes < 0)
    remainingMinutes = 0;

  // If trigger reached and not yet triggered, move servo to 180° once
  if ((long long)now >= (long long)trigger_date && !servoTriggered)
  {
    myservo.attach(SERVO_PIN);
    myservo.write(170);
    servoTriggered = true;
    delay(1000);
    myservo.detach();
  }

  // Display time and remaining minutes
  tft.fillRect(0, 0, 240, 80, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(5, 5);
  tft.print(timestr);

  tft.setCursor(5, 35);
  tft.print("rem: ");
  tft.print(remainingMinutes);
  tft.print(" min");

  // Feed watchdog and notify on display
  esp_task_wdt_reset();
  showMessage("WDT fed", 130, TFT_GREEN, 1);

  delay(1000);
}