// Clean Adafruit ST7789 test sketch
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <WiFi.h>
#include <time.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
// GPS parsing + low-level UART driver
#include <TinyGPSPlus.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

// Pins from the TTGO config / discovered earlier
static const int TFT_MOSI = 19;
static const int TFT_SCLK = 18;
static const int TFT_CS = 5;
static const int TFT_DC = 16;
static const int TFT_RST = 23;
static const int TFT_BL = 4;

// Buzzer wiring: connect a (passive) buzzer or speaker between these two GPIO pins
// (no GND needed). The code toggles the pins in opposite phase to increase
// the effective voltage swing across the buzzer for stronger sound.
// Change these pin numbers to match your wiring.
static const int BUZZER_PIN_A = 25;
static const int BUZZER_PIN_B = 26;

// Create ST7789 instance using hardware SPI
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);

// Watchdog timeout (seconds)
#define WDT_TIMEOUT 10

// Screen and shutdown management
unsigned long bootMillis = 0;
bool screenIsOn = true;
bool shutdownScheduled = false;
unsigned long shutdownTimeMillis = 0;
const unsigned long SCREEN_OFF_AFTER_BOOT_MS = 60000UL;        // 1 minute
const unsigned long SHUTDOWN_DELAY_AFTER_TRIGGER_MS = 60000UL; // 1 minute after remaining==0

int w, h;

// --- GPS / UART1 handling (interrupt-driven, TinyGPSPlus) ---
uart_port_t GPS_UART = UART_NUM_1; // use UART1
static const int GPS_RX_PIN = 21;  // user requested mapping
static const int GPS_TX_PIN = 22;
static const int GPS_BAUD = 9600;

TinyGPSPlus gps;

typedef struct
{
  double lat;
  double lng;
  double alt;
  double speed_kmh;
  bool valid;
  char raw[128];
  int sats;
} GPSMsg;

// --- Radar detection structures and helpers ---
#include "radar.h"
static RadarAlertManager radarManager;

// Radar alert state: set by display task, consumed by loop to produce timed beeps
static volatile bool radarAlertActive = false;
static unsigned long radarDesiredIntervalMs = 0;
static unsigned long radarDesiredDurationMs = 0;
static uint32_t radarDesiredFreqHz = 0;
static volatile bool radarEnteredZone = false;

// Non-blocking beep state (managed in loop)
static unsigned long lastBeepTrigger = 0;
static unsigned long beepOnUntil = 0;
static bool beepCurrentlyOn = false;

static QueueHandle_t gpsQueue = NULL;   // queue for parsed GPS messages (to display)
static QueueHandle_t uart_queue = NULL; // uart event queue
// Task handles (used when pinning tasks to cores)
static TaskHandle_t uartTaskHandle = NULL;
static TaskHandle_t displayTaskHandle = NULL;

// Forward declarations
static void uart_event_task(void *pvParameters);
static void gps_display_task(void *pvParameters);

// GPS driver functions (implemented in driver_gps6mv2.cpp)
#include "driver_gps6mv2.h"
// Buzzer driver
#include "driver_buzzer.h"

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

// ISR functions: keep them tiny and only set volatile flags.
void IRAM_ATTR button1ISR()
{
  button1Pressed = true;
}
void IRAM_ATTR button2ISR()
{
  button2Pressed = true;
}

// Determine radar alerts from current position + speed. Chooses highest-severity alert
static void updateRadarAlerts(double lat, double lng, double speedKmh, int sats = -1)
{
  Serial.printf("%.6f, %.6f, %.1f, %d\n", lat, lng, speedKmh, sats);
  RadarAlertResult r = radarManager.update(lat, lng, speedKmh);

  if (!r.inZone || r.severity == 0)
  {
    radarAlertActive = false;
    return;
  }

  // if severity changed, reset last beep time to trigger immediate beep
  if (!radarAlertActive || r.intervalMs != radarDesiredIntervalMs ||
      r.durationMs != radarDesiredDurationMs || r.freqHz != radarDesiredFreqHz)
  {
    stopTone();
    lastBeepTrigger = 0;
  }

  radarAlertActive = true;
  radarDesiredIntervalMs = r.intervalMs;
  radarDesiredDurationMs = r.durationMs;
  radarDesiredFreqHz = r.freqHz;
}

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

// UART event task: runs when UART driver posts an event (interrupt-driven)
static void uart_event_task(void *pvParameters)
{
  uart_event_t event;
  uint8_t dtmp[256];
  for (;;)
  {
    // Block until an event is available
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
    {
      if (event.type == UART_DATA)
      {
        int len = uart_read_bytes(GPS_UART, dtmp, event.size, portMAX_DELAY);
        // Debug: print RX activity to Serial console
        // Serial.printf("[GPS UART] DATA received: %d bytes\n", len);
        // Serial.print("[GPS UART] HEX: ");
        /*for (int i = 0; i < len; ++i)
        {
          Serial.printf("%02X ", dtmp[i]);
        }*/
        /*Serial.println();
        Serial.print("[GPS UART] STR: ");
        Serial.write(dtmp, len);
        Serial.println();*/
        for (int i = 0; i < len; ++i)
        {
          gps.encode(dtmp[i]);
        }

        // Prepare debug/raw string (truncate if too long)
        GPSMsg msg;
        size_t copy_len = (size_t)len;
        if (copy_len >= sizeof(msg.raw))
          copy_len = sizeof(msg.raw) - 1;
        if (copy_len > 0)
        {
          memcpy(msg.raw, dtmp, copy_len);
        }
        msg.raw[copy_len] = '\0';

        // If we have new information, fill numeric fields and send it to the display task
        msg.valid = gps.location.isValid();
        msg.lat = msg.valid ? gps.location.lat() : 0.0;
        msg.lng = msg.valid ? gps.location.lng() : 0.0;
        msg.alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
        msg.speed_kmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
        // Satellite count (may be 0 if no fix)
        msg.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
        xQueueSend(gpsQueue, &msg, 0);
        /*if (xQueueSend(gpsQueue, &msg, 0) != pdTRUE)
        {
          Serial.println("[GPS UART] Warning: gpsQueue send failed");
        }
        else
        {
          Serial.println("[GPS UART] Message queued for display");
        }*/
      }
      else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL)
      {
        // Overflow, flush input
        // Serial.println("[GPS UART] FIFO overflow / buffer full - flushing");
        uart_flush_input(GPS_UART);
        xQueueReset(uart_queue);
      }
      // other event types are ignored for brevity
    }
  }
}

// Display task: blocks on gpsQueue and updates the display when a new GPSMsg arrives
static void gps_display_task(void *pvParameters)
{
  GPSMsg msg;
  char linebuf[128];
  for (;;)
  {
    if (xQueueReceive(gpsQueue, &msg, portMAX_DELAY) == pdTRUE)
    {
      if (!msg.valid)
      {
        char status[64];
        snprintf(status, sizeof(status), "Fix: %s  Sats: %d", msg.valid ? "YES" : "NO", msg.sats);
        showMessage(status, 92, msg.valid ? ST77XX_GREEN : ST77XX_YELLOW, 1);
        /*
                showMessage("GPS: no fix", 20, ST77XX_RED, 1);
                // Serial.println("GPS: no fix");
                //  still show raw debug even if no fix
                char tmp[64];
                snprintf(tmp, sizeof(tmp), "No fix - sats: %d", msg.sats);
                showMessage(tmp, 20, ST77XX_YELLOW, 1);
        */
        // showMessage(msg.raw, h - 18, ST77XX_YELLOW, 1);
        continue;
      }

      // Position
      snprintf(linebuf, sizeof(linebuf), "Lat: %.6f", msg.lat);
      showMessage(linebuf, 20, ST77XX_WHITE, 1);
      snprintf(linebuf, sizeof(linebuf), "Lon: %.6f", msg.lng);
      showMessage(linebuf, 36, ST77XX_WHITE, 1);

      // Altitude in meters
      snprintf(linebuf, sizeof(linebuf), "Alt: %.1f m", msg.alt);
      showMessage(linebuf, 56, ST77XX_WHITE, 1);

      // Speed in km/h
      snprintf(linebuf, sizeof(linebuf), "Speed: %.1f km/h", msg.speed_kmh);
      showMessage(linebuf, 76, ST77XX_WHITE, 1);
      // Debug raw transfer (bottom of screen)
      /*if (msg.raw[0] != '\0')
      {
        showMessage(msg.raw, h - 18, ST77XX_YELLOW, 1);
      }*/

      // Show fix status + satellite count under speed
      char status[64];
      snprintf(status, sizeof(status), "Fix: %s  Sats: %d", msg.valid ? "YES" : "NO", msg.sats);
      showMessage(status, 92, msg.valid ? ST77XX_GREEN : ST77XX_YELLOW, 1);

      // Update radar alert logic (sets global desired beep interval/duration)
      // updateRadarAlerts(msg.lat, msg.lng, msg.speed_kmh, msg.sats);
      // Yield briefly so the idle task on the other core can run and the watchdog
      // won't be starved by long display operations. Adjust the delay as needed.
      vTaskDelay(2);
    }
  }
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
  // Serial.println("Backlight set HIGH on pin " + String(TFT_BL));

  // Init display with confirmed resolution 240x135 and draw 2px border
  // Serial.println("Init display 240x135 and draw 2px border...");
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

  // Serial.printf("Border drawn. display width=%d height=%d\n", w, h);

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
    delay(1000);
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
  // Initialize buzzer driver (connect passive buzzer between these two pins)
  buzzerInit(BUZZER_PIN_A, BUZZER_PIN_B);
  // Optional quick test beep (uncomment to hear a short tone on boot)
  beep(2000, 150);
  // --- Initialize UART1 for GPS (interrupt-driven) ---
  {
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(GPS_UART, &uart_config);
    uart_set_pin(GPS_UART, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // install driver with event queue (no busy-polling)
    const int uart_buffer_size = 2048;
    uart_driver_install(GPS_UART, uart_buffer_size, 0, 20, &uart_queue, 0);
    Serial.printf("GPS UART initialized: UART=%d RX=%d TX=%d Baud=%d\n", (int)GPS_UART, GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
    // Configure GPS module update rate to 5Hz (200ms) by sending UBX-CFG-RATE
    delay(50);
    sendUbxCfgRate5Hz();
    delay(100);
    // Now request GPS module to switch to 115200 baud (must talk at 9600 first)
    sendUbxCfgPrt115200();
    delay(200);
    // Reconfigure ESP32 UART to 115200 to match module
    if (uart_set_baudrate != NULL)
    {
      // try to set baudrate via API if available
      uart_set_baudrate(GPS_UART, 115200);
      Serial.println("[GPS UART] Reconfigured ESP UART to 115200 via uart_set_baudrate");
    }
    else
    {
      // fallback: update params and continue
      uart_config_t newcfg = {
          .baud_rate = 115200,
          .data_bits = UART_DATA_8_BITS,
          .parity = UART_PARITY_DISABLE,
          .stop_bits = UART_STOP_BITS_1,
          .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      };
      uart_param_config(GPS_UART, &newcfg);
      Serial.println("[GPS UART] Reconfigured ESP UART to 115200 via uart_param_config");
    }

    // create queue for parsed GPS messages
    gpsQueue = xQueueCreate(4, sizeof(GPSMsg));

    // create a task to handle UART events (runs when data arrives)
    // pin the UART event task to CPU 0 so it won't interfere with Arduino `loop()` on CPU 1
    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 4096, NULL, 12, &uartTaskHandle, 0);
    // create a task to display GPS data (blocks on gpsQueue)
    // pin the display task to CPU 0 or 1 depending on your responsiveness needs.
    // Here we pin the display to CPU 0 to keep long display operations off the Arduino core (CPU 1)
    xTaskCreatePinnedToCore(gps_display_task, "gps_display_task", 8192, NULL, 1, &displayTaskHandle, 0);
  }
  // --- End migrated logic from main2 setup ---
  // Clear only the interior so the border drawn earlier remains visible
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 0, w - 1, h - 1, ST77XX_WHITE);

  // record boot time for screen management
  bootMillis = millis();
  screenIsOn = true;
}

static const double bp1_test_coords[][4] = {
    {28.070220, -16.707558, 61.5, 6},
    {28.070220, -16.707558, 61.5, 6},
    {28.070220, -16.707558, 61.5, 6},
    {28.070220, -16.707558, 61.5, 6},
    {28.070225, -16.707524, 61.2, 6},
    {28.070225, -16.707524, 61.2, 6},
    {28.070225, -16.707524, 61.2, 6},
    {28.070225, -16.707524, 61.2, 6},
    {28.070230, -16.707490, 61.2, 6},
    {28.070230, -16.707490, 61.2, 6},
    {28.070230, -16.707490, 61.2, 6},
    {28.070230, -16.707490, 61.2, 6},
    {28.070236, -16.707456, 61.1, 6},
    {28.070236, -16.707456, 61.1, 6},
    {28.070236, -16.707456, 61.1, 6},
    {28.070236, -16.707456, 61.1, 6},
    {28.070244, -16.707424, 60.8, 6},
    {28.070244, -16.707424, 60.8, 6},
    {28.070244, -16.707424, 60.8, 6},
    {28.070244, -16.707424, 60.8, 6},
    {28.070250, -16.707390, 61.0, 6},
    {28.070250, -16.707390, 61.0, 6},
    {28.070250, -16.707390, 61.0, 6},
    {28.070250, -16.707390, 61.0, 6},
    {28.070258, -16.707358, 60.7, 6},
    {28.070258, -16.707358, 60.7, 6},
    {28.070258, -16.707358, 60.7, 6},
    {28.070258, -16.707358, 60.7, 6},
    {28.070265, -16.707324, 60.7, 6},
    {28.070265, -16.707324, 60.7, 6},
    {28.070265, -16.707324, 60.7, 6},
    {28.070265, -16.707324, 60.7, 6},
    {28.070273, -16.707291, 60.7, 6},
    {28.070273, -16.707291, 60.7, 6},
    {28.070273, -16.707291, 60.7, 6},
    {28.070273, -16.707291, 60.7, 6},
    {28.070281, -16.707258, 60.5, 6},
    {28.070281, -16.707258, 60.5, 6},
    {28.070281, -16.707258, 60.5, 6},
    {28.070281, -16.707258, 60.5, 6},
    {28.070289, -16.707225, 60.3, 6},
    {28.070289, -16.707225, 60.3, 6},
    {28.070289, -16.707225, 60.3, 6},
    {28.070289, -16.707225, 60.3, 6},
    {28.070297, -16.707193, 60.0, 6},
    {28.070297, -16.707193, 60.0, 6},
    {28.070297, -16.707193, 60.0, 6},
    {28.070297, -16.707193, 60.0, 6},
    {28.070306, -16.707161, 59.6, 6},
    {28.070306, -16.707161, 59.6, 6},
    {28.070306, -16.707161, 59.6, 6},
    {28.070306, -16.707161, 59.6, 6},
    {28.070314, -16.707128, 59.6, 6},
    {28.070314, -16.707128, 59.6, 6},
    {28.070314, -16.707128, 59.6, 6},
    {28.070314, -16.707128, 59.6, 6},
    {28.070323, -16.707097, 59.3, 6},
    {28.070323, -16.707097, 59.3, 6},
    {28.070323, -16.707097, 59.3, 6},
    {28.070323, -16.707097, 59.3, 6},
    {28.070332, -16.707065, 59.2, 6},
    {28.070332, -16.707065, 59.2, 6},
    {28.070332, -16.707065, 59.2, 6},
    {28.070332, -16.707065, 59.2, 6},
    {28.070341, -16.707033, 58.9, 6},
    {28.070341, -16.707033, 58.9, 6},
    {28.070341, -16.707033, 58.9, 6},
    {28.070341, -16.707033, 58.9, 6},
    {28.070350, -16.707002, 58.8, 6},
    {28.070350, -16.707002, 58.8, 6},
    {28.070350, -16.707002, 58.8, 6},
    {28.070350, -16.707002, 58.8, 6},
    {28.070358, -16.706969, 59.2, 6},
    {28.070358, -16.706969, 59.2, 6},
    {28.070358, -16.706969, 59.2, 6},
    {28.070358, -16.706969, 59.2, 6},
    {28.070367, -16.706937, 59.1, 6},
    {28.070367, -16.706937, 59.1, 6},
    {28.070367, -16.706937, 59.1, 6},
    {28.070367, -16.706937, 59.1, 6},
    {28.070377, -16.706906, 59.2, 6},
    {28.070377, -16.706906, 59.2, 6},
    {28.070377, -16.706906, 59.2, 6},
    {28.070377, -16.706906, 59.2, 6},
    {28.070390, -16.706878, 58.3, 6},
    {28.070390, -16.706878, 58.3, 6},
    {28.070390, -16.706878, 58.3, 6},
    {28.070390, -16.706878, 58.3, 6},
    {28.070390, -16.706878, 58.3, 6},
    {28.070402, -16.706848, 58.1, 6},
    {28.070402, -16.706848, 58.1, 6},
    {28.070402, -16.706848, 58.1, 6},
    {28.070402, -16.706848, 58.1, 6},
    {28.070402, -16.706848, 58.1, 6},
    {28.070413, -16.706818, 57.7, 6},
    {28.070413, -16.706818, 57.7, 6},
    {28.070413, -16.706818, 57.7, 6},
    {28.070413, -16.706818, 57.7, 6},
    {28.070424, -16.706789, 57.3, 6},
    {28.070424, -16.706789, 57.3, 6},
    {28.070424, -16.706789, 57.3, 6},
    {28.070424, -16.706789, 57.3, 6},
    {28.070434, -16.706758, 57.7, 6},
    {28.070434, -16.706758, 57.7, 6},
    {28.070434, -16.706758, 57.7, 6},
    {28.070434, -16.706758, 57.7, 6},
    {28.070434, -16.706758, 57.7, 6},
    {28.070445, -16.706727, 58.0, 6},
    {28.070445, -16.706727, 58.0, 6},
    {28.070445, -16.706727, 58.0, 6},
    {28.070445, -16.706727, 58.0, 6},
    {28.070455, -16.706696, 58.4, 6},
    {28.070455, -16.706696, 58.4, 6},
    {28.070455, -16.706696, 58.4, 6},
    {28.070455, -16.706696, 58.4, 6},
    {28.070455, -16.706696, 58.4, 6},
    {28.070466, -16.706665, 58.5, 6},
    {28.070466, -16.706665, 58.5, 6},
    {28.070466, -16.706665, 58.5, 6},
    {28.070466, -16.706665, 58.5, 6},
    {28.070476, -16.706634, 58.4, 6},
    {28.070476, -16.706634, 58.4, 6},
    {28.070476, -16.706634, 58.4, 6},
    {28.070476, -16.706634, 58.4, 6},
    {28.070476, -16.706634, 58.4, 6},
    {28.070485, -16.706601, 58.6, 6},
    {28.070485, -16.706601, 58.6, 6},
    {28.070485, -16.706601, 58.6, 6},
    {28.070485, -16.706601, 58.6, 6},
    {28.070496, -16.706571, 58.5, 6},
    {28.070496, -16.706571, 58.5, 6},
    {28.070496, -16.706571, 58.5, 6},
    {28.070496, -16.706571, 58.5, 6},
    {28.070496, -16.706571, 58.5, 6},
    {28.070508, -16.706541, 58.8, 6},
    {28.070508, -16.706541, 58.8, 6},
    {28.070508, -16.706541, 58.8, 6},
    {28.070508, -16.706541, 58.8, 6},
    {28.070519, -16.706512, 58.6, 6},
    {28.070519, -16.706512, 58.6, 6},
    {28.070519, -16.706512, 58.6, 6},
    {28.070519, -16.706512, 58.6, 6},
    {28.070531, -16.706482, 58.2, 6},
    {28.070531, -16.706482, 58.2, 5},
    {28.070531, -16.706482, 58.2, 5},
    {28.070531, -16.706482, 58.2, 5},
    {28.070539, -16.706450, 58.1, 5},
    {28.070539, -16.706450, 58.1, 5},
    {28.070539, -16.706450, 58.1, 5},
    {28.070539, -16.706450, 58.1, 5},
    {28.070549, -16.706420, 57.7, 5},
    {28.070549, -16.706420, 57.7, 5},
    {28.070549, -16.706420, 57.7, 5},
    {28.070549, -16.706420, 57.7, 5},
    {28.070559, -16.706389, 57.8, 5},
    {28.070559, -16.706389, 57.8, 5},
    {28.070559, -16.706389, 57.8, 5},
    {28.070559, -16.706389, 57.8, 5},
    {28.070567, -16.706359, 57.2, 5},
    {28.070567, -16.706359, 57.2, 5},
    {28.070567, -16.706359, 57.2, 5},
    {28.070567, -16.706359, 57.2, 5},
    {28.070576, -16.706328, 57.5, 5},
    {28.070576, -16.706328, 57.5, 5},
    {28.070576, -16.706328, 57.5, 5},
    {28.070576, -16.706328, 57.5, 5},
    {28.070584, -16.706297, 57.2, 5},
    {28.070584, -16.706297, 57.2, 5},
    {28.070584, -16.706297, 57.2, 5},
    {28.070584, -16.706297, 57.2, 5},
    {28.070593, -16.706266, 57.3, 5},
    {28.070593, -16.706266, 57.3, 5},
    {28.070593, -16.706266, 57.3, 5},
    {28.070593, -16.706266, 57.3, 5},
    {28.070601, -16.706235, 57.1, 5},
    {28.070601, -16.706235, 57.1, 5},
    {28.070601, -16.706235, 57.1, 5},
    {28.070601, -16.706235, 57.1, 5},
    {28.070611, -16.706204, 57.0, 5},
    {28.070611, -16.706204, 57.0, 5},
    {28.070611, -16.706204, 57.0, 5},
    {28.070611, -16.706204, 57.0, 5},
    {28.070620, -16.706173, 57.1, 5},
    {28.070620, -16.706173, 57.1, 5},
};

static bool test_running = false;
static int test_step = 0;
static unsigned long test_last_update_ms = 0;

uint8_t test(bool bp_status, const double test_coords[][4])
{
  if (test_running == false && bp_status == true)
  {
    test_last_update_ms = millis();
    test_running = true;
  }
  else if (test_running == true && millis() - test_last_update_ms >= 500)
  {
    test_last_update_ms = millis();
    updateRadarAlerts(test_coords[test_step][0], test_coords[test_step][1], test_coords[test_step][2], , test_coords[test_step][3]);
    test_step++;
    if (test_step >= 187)
    {
      test_step = 0;
      test_running = false;
      return 0;
    }
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "test step: %d", test_step);
    showMessage(tmp, h / 2 - 14, ST77XX_WHITE, 1);
  }
  return 1;
}

static bool test_app_running = false;

void test_app()
{
  static uint8_t test_num = 0;

  if (test_app_running == false)
    return;

  /* switch (test_num)
  {
  case 0: */
  showMessage("Test 0: enter radar zone", h / 2 - 10, ST77XX_WHITE, 1);
  if (test(true, bp1_test_coords) == 0)
    test_num++;
  /*  break;
 case 1:
   showMessage("Test 1: stay below limit", h / 2 - 10, ST77XX_WHITE, 1);
   if (test(true, bp1_test_coords, bp2_test_speeds_kmh) == 0)
     test_num++;
   break;
 case 2:
   showMessage("Test 2: enter radar zone at max speed - 1", h / 2 - 10, ST77XX_WHITE, 1);
   if (test(true, bp1_test_coords, bp3_test_speeds_kmh) == 0)
     test_num++;
   break;
 default:
   showMessage("Test complete", h / 2 - 10, ST77XX_WHITE, 1);
   test_app_running = false;
   test_num = 0;
   break;
 } */
}

void loop()
{

  esp_task_wdt_reset();

  test_app();

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
      test_app_running = true;
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

      test_app_running = false;
    }
    else
    {
      button2Pressed = false;
    }
  }

  // Manage radar beeps non-blocking (timed beeps)
  unsigned long now = millis();

  if (radarAlertActive)
  {
    if (!beepCurrentlyOn)
    {
      if (now - lastBeepTrigger >= radarDesiredIntervalMs)
      {
        startTone(radarDesiredFreqHz);
        beepCurrentlyOn = true;
        beepOnUntil = now + radarDesiredDurationMs;
        lastBeepTrigger = now;
      }
    }
    else
    {
      if (now >= beepOnUntil)
      {
        stopTone();
        beepCurrentlyOn = false;
      }
    }
  }
  else
  {
    if (beepCurrentlyOn)
    {
      stopTone();
      beepCurrentlyOn = false;
    }
    // reset trigger so we don't immediately beep when alert appears
    lastBeepTrigger = now;
  }

  // Short delay for loop granularity (100ms) - keeps responsiveness for beeps
  delay(100);
}
