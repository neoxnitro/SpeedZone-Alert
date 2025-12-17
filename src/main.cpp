// Clean Adafruit ST7789 test sketch
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <WiFi.h>
#include <time.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
// MQTT
#include <PubSubClient.h>

// Pins from the TTGO config / discovered earlier
static const int TFT_MOSI = 19;
static const int TFT_SCLK = 18;
static const int TFT_CS = 5;
static const int TFT_DC = 16;
static const int TFT_RST = 23;
static const int TFT_BL = 4;

// Create ST7789 instance using hardware SPI
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);

// --- From main2: WiFi, servo and watchdog configuration ---
// WiFi credentials
const char *ssid1 = "Livebox6-1CB6-24G";
const char *pass1 = "dxGVxr6Kb6LV";
const char *ssid2 = "TP-Link_25FE";
const char *pass2 = "37522872";

// Set your trigger epoch here (seconds since 1970)
// NOTE: became a variable so button handler can update it at runtime.
time_t trigger_date = 1774835200UL;

// Watchdog timeout (seconds)
#define WDT_TIMEOUT 10

// Servo configuration
const int SERVO_PIN = 13;
Servo myservo;
bool servoTriggered = false;
// Track current servo position (degrees). Initialized to 10 in setup.
int currentServoDeg = 10;

// MQTT globals
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const char *mqtt_server = "192.168.1.52";
const char *mqttTopic = "/esp32tto/status";
unsigned long lastHourlyPublish = 0;
unsigned long lastPublishValue = 0;
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 10000UL; // try reconnect every 10s
bool servoPublished = false;                           // ensure single publish when servoTriggered becomes true
bool bootStatusPublished = false;                      // publish '0' once after first successful MQTT connection

// Screen and shutdown management
unsigned long bootMillis = 0;
bool screenIsOn = true;
bool shutdownScheduled = false;
unsigned long shutdownTimeMillis = 0;
const unsigned long SCREEN_OFF_AFTER_BOOT_MS = 60000UL;        // 1 minute
const unsigned long SHUTDOWN_DELAY_AFTER_TRIGGER_MS = 60000UL; // 1 minute after remaining==0

int w, h;

// Button pins (change these to match your board wiring)
// For TTGO boards you may need to adjust these pins to the actual buttons used.
static const int BUTTON1_PIN = 0;  // button 1: set trigger_date = now + 1 minute and wake screen
static const int BUTTON2_PIN = 35; // button 2: wake screen for 1 minute

// Volatile flags set from ISRs
volatile bool button1Pressed = false;
volatile bool button2Pressed = false;

// Debounce and wake-expiration tracking (non-volatile OK; updated in loop)
unsigned long lastButton1Handled = 0;
unsigned long lastButton2Handled = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 250;
unsigned long wakeExpireMillis = 0; // if > millis() keep screen on for this long

// Helper to draw messages using Adafruit GFX (replaces TFT_eSPI showMessage)
void showMessage(const char *msg, int y, uint16_t color = ST77XX_WHITE, uint8_t size = 2)
{
  // Use getTextBounds to compute exact bounding box and clear it before printing
  tft.setTextSize(size);
  int16_t x1, y1;
  uint16_t bw, bh;
  const int padding = 4;
  // we'll print starting at x=5 to leave a small margin
  const int16_t cx = 5;
  tft.getTextBounds(msg, cx, y, &x1, &y1, &bw, &bh);
  int16_t rx = cx - padding;
  int16_t ry = y - padding;
  if (rx < 0)
    rx = 0;
  if (ry < 0)
    ry = 0;
  uint16_t rw = bw + padding * 2;
  uint16_t rh = bh + padding * 2;
  // Ensure we don't overflow screen
  int16_t sw = tft.width();
  int16_t sh = tft.height();
  if (rx + rw > sw)
    rw = sw - rx;
  if (ry + rh > sh)
    rh = sh - ry;
  tft.fillRect(rx, ry, rw, rh, ST77XX_BLACK);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(cx, y);
  tft.print(msg);
}

// ISR functions: keep them tiny and only set volatile flags.
void IRAM_ATTR button1ISR()
{
  button1Pressed = true;
}

void IRAM_ATTR button2ISR()
{
  button2Pressed = true;
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  // Initialize SPI on the pins the board uses (SCLK, MISO, MOSI)
  // MISO not used by the display; set to -1
  SPI.begin(TFT_SCLK, -1, TFT_MOSI);

  // Turn on backlight if present
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.println("Backlight set HIGH on pin " + String(TFT_BL));

  // Init display with confirmed resolution 240x135 and draw 2px border
  Serial.println("Init display 240x135 and draw 2px border...");
  // Use the confirmed physical resolution
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // Get width/height from library (safe way to match current init/rotation)
  w = tft.width();
  h = tft.height();

  // Draw outer 1px border (use width-1 and height-1 because drawRect uses width,height)
  tft.drawRect(0, 0, w - 1, h - 1, ST77XX_WHITE);

  // Draw verification text
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.println("Bonjour TTGO!");

  tft.setTextSize(1);
  tft.setCursor(8, h - 20);
  tft.print("Res: ");
  tft.print(w);
  tft.print("x");
  tft.println(h);

  Serial.printf("Border drawn. display width=%d height=%d\n", w, h);

  // --- Begin migrated logic from main2 setup ---
  showMessage("Booting ...", 10, ST77XX_WHITE, 2);
  delay(1000);

  // Clear only the interior (preserve 1px white border)
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 0, w - 1, h - 1, ST77XX_WHITE);

  // Initialize and enable watchdog for this task
  esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT, true);
  if (err == ESP_OK)
  {
    esp_task_wdt_add(NULL); // add current thread to WDT
    showMessage("Watchdog enabled", 100, ST77XX_WHITE, 2);
    delay(200);
  }
  else
  {
    showMessage("WDT init failed", 100, ST77XX_RED, 2);
    delay(1000);
  }
  // Configure buttons and attach interrupts
  // Use internal pullups and falling-edge trigger (buttons to GND)
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), button2ISR, FALLING);
  // --- End migrated logic from main2 setup ---
  // Clear only the interior so the border drawn earlier remains visible
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 0, w - 1, h - 1, ST77XX_WHITE);

  // record boot time for screen management
  bootMillis = millis();
  screenIsOn = true;
}

void loop()
{

  // Handle button events set by ISRs (debounced in loop)
  if (button1Pressed)
  {
    // debounce
    unsigned long nowMs = millis();
    if (nowMs - lastButton1Handled >= BUTTON_DEBOUNCE_MS)
    {
      lastButton1Handled = nowMs;
      // clear flag early
      button1Pressed = false;
      // xx
      showMessage("Button1: wake 1m", h / 2 - 10, ST77XX_WHITE, 1);
    }
    else
    {
      // clear spurious flag
      button1Pressed = false;
    }
  }

  if (button2Pressed)
  {
    unsigned long nowMs = millis();
    if (nowMs - lastButton2Handled >= BUTTON_DEBOUNCE_MS)
    {
      lastButton2Handled = nowMs;
      button2Pressed = false;
      // yy
      showMessage("Button2: wake 1m", h / 2 - 10, ST77XX_WHITE, 1);
    }
    else
    {
      button2Pressed = false;
    }
  }
  esp_task_wdt_reset();

  delay(1000);
}