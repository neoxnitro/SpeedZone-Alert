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
static const uart_port_t GPS_UART = UART_NUM_1; // use UART1
static const int GPS_RX_PIN = 21;               // user requested mapping
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

static QueueHandle_t gpsQueue = NULL;   // queue for parsed GPS messages (to display)
static QueueHandle_t uart_queue = NULL; // uart event queue

// Forward declarations
static void uart_event_task(void *pvParameters);
static void gps_display_task(void *pvParameters);
static void sendUbxCfgRate5Hz();
static void sendUbxCfgPrt115200();
// Buzzer control
static void startTone(uint32_t freqHz);
static void stopTone();
static void beep(uint32_t freqHz, uint32_t durationMs);

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

// --- Buzzer timer and state ---
static hw_timer_t *buzzerTimer = NULL;
static volatile bool buzzerState = false;

// Timer ISR toggles the two buzzer pins in opposite phase to increase swing.
void IRAM_ATTR onBuzzerTimer()
{
  buzzerState = !buzzerState;
  gpio_set_level((gpio_num_t)BUZZER_PIN_A, buzzerState ? 1 : 0);
  gpio_set_level((gpio_num_t)BUZZER_PIN_B, buzzerState ? 0 : 1);
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
        Serial.printf("[GPS UART] DATA received: %d bytes\n", len);
        Serial.print("[GPS UART] HEX: ");
        for (int i = 0; i < len; ++i)
        {
          Serial.printf("%02X ", dtmp[i]);
        }
        Serial.println();
        Serial.print("[GPS UART] STR: ");
        Serial.write(dtmp, len);
        Serial.println();
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
        if (xQueueSend(gpsQueue, &msg, 0) != pdTRUE)
        {
          Serial.println("[GPS UART] Warning: gpsQueue send failed");
        }
        else
        {
          Serial.println("[GPS UART] Message queued for display");
        }
      }
      else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL)
      {
        // Overflow, flush input
        Serial.println("[GPS UART] FIFO overflow / buffer full - flushing");
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
        showMessage("GPS: no fix", 20, ST77XX_RED, 1);
        // still show raw debug even if no fix
        char tmp[64];
        snprintf(tmp, sizeof(tmp), "No fix - sats: %d", msg.sats);
        showMessage(tmp, 20, ST77XX_YELLOW, 1);
        showMessage(msg.raw, h - 18, ST77XX_YELLOW, 1);
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
      if (msg.raw[0] != '\0')
      {
        showMessage(msg.raw, h - 18, ST77XX_YELLOW, 1);
      }

      // Show fix status + satellite count under speed
      char status[64];
      snprintf(status, sizeof(status), "Fix: %s  Sats: %d", msg.valid ? "YES" : "NO", msg.sats);
      showMessage(status, 92, msg.valid ? ST77XX_GREEN : ST77XX_YELLOW, 1);
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
  // Configure buzzer pins (outputs). Connect passive buzzer between these two pins.
  pinMode(BUZZER_PIN_A, OUTPUT);
  pinMode(BUZZER_PIN_B, OUTPUT);
  digitalWrite(BUZZER_PIN_A, LOW);
  digitalWrite(BUZZER_PIN_B, LOW);
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
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    // create a task to display GPS data (blocks on gpsQueue)
    xTaskCreate(gps_display_task, "gps_display_task", 4096, NULL, 1, NULL);
  }
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

// Send UBX CFG-RATE message to set update rate to 5Hz (measurement rate = 200 ms)
static void sendUbxCfgRate5Hz()
{
  // UBX-CFG-RATE payload: measRate (ms, uint16), navRate (cycles, uint16), timeRef (uint16)
  // For 5Hz: measRate = 200, navRate = 1, timeRef = 0
  const uint8_t ubx[] = {
      0xB5, 0x62, // header
      0x06, 0x08, // class, id (CFG-RATE)
      0x06, 0x00, // length = 6
      0xC8, 0x00, // measRate = 200 ms (0x00C8 -> little-endian 0xC8 0x00)
      0x01, 0x00, // navRate = 1
      0x00, 0x00, // timeRef = 0 (UTC)
      0xDD, 0x68  // checksum CK_A, CK_B (precomputed for above payload)
  };

  int len = sizeof(ubx);
  int written = uart_write_bytes(GPS_UART, (const char *)ubx, len);
  Serial.printf("[GPS UBX] Sent CFG-RATE 5Hz (%d bytes) -> written=%d\n", len, written);
}

// Send UBX CFG-PRT to change GPS UART baudrate to 115200 (must be sent at current baud)
static void sendUbxCfgPrt115200()
{
  const uint16_t payload_len = 20; // use 20 bytes payload (pad zeros if needed)
  uint8_t payload[payload_len];
  memset(payload, 0, sizeof(payload));

  // Build payload fields (little endian as required)
  payload[0] = 0x01; // portID = 1 (UART)
  payload[1] = 0x00; // reserved
  payload[2] = 0x00; // txReady LSB
  payload[3] = 0x00; // txReady MSB

  // mode (4 bytes) - use 0x000008D0 (8N1 typical flags)
  payload[4] = 0xD0;
  payload[5] = 0x08;
  payload[6] = 0x00;
  payload[7] = 0x00;

  // baudRate (4 bytes) little endian for 115200 = 0x0001C200 -> 00 C2 01 00
  payload[8] = 0x00;
  payload[9] = 0xC2;
  payload[10] = 0x01;
  payload[11] = 0x00;

  // inProtoMask (2 bytes) = 0x0007 (UBX+NMEA)
  payload[12] = 0x07;
  payload[13] = 0x00;
  // outProtoMask (2 bytes)
  payload[14] = 0x07;
  payload[15] = 0x00;

  // remaining bytes left zero (reserved)

  // Build message buffer: header + class,id + length + payload + checksum
  const uint8_t cls = 0x06;
  const uint8_t id = 0x00;
  uint8_t msg[6 + payload_len + 2];
  int idx = 0;
  msg[idx++] = 0xB5;
  msg[idx++] = 0x62;
  msg[idx++] = cls;
  msg[idx++] = id;
  msg[idx++] = (uint8_t)(payload_len & 0xFF);
  msg[idx++] = (uint8_t)((payload_len >> 8) & 0xFF);
  memcpy(&msg[idx], payload, payload_len);
  idx += payload_len;

  // compute checksum over class,id,lengthL, lengthH and payload
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < 6 + payload_len; ++i)
  {
    ck_a = ck_a + msg[i];
    ck_b = ck_b + ck_a;
  }
  msg[idx++] = ck_a;
  msg[idx++] = ck_b;

  int tosend = idx;
  int written = uart_write_bytes(GPS_UART, (const char *)msg, tosend);
  Serial.printf("[GPS UBX] Sent CFG-PRT (change baud->115200) %d bytes, written=%d\n", tosend, written);
}

// --- Buzzer control functions ---
// Start a square wave on the two pins with given frequency (Hz)
static void startTone(uint32_t freqHz)
{
  if (freqHz == 0)
    return;
  uint32_t halfPeriodUs = 500000UL / freqHz; // half period in microseconds
  if (!buzzerTimer)
  {
    // Use timer 0, prescaler 80 -> 1 tick = 1 microsecond
    buzzerTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(buzzerTimer, &onBuzzerTimer, true);
  }
  timerAlarmWrite(buzzerTimer, halfPeriodUs, true);
  timerAlarmEnable(buzzerTimer);
}

static void stopTone()
{
  if (buzzerTimer)
  {
    timerAlarmDisable(buzzerTimer);
  }
  gpio_set_level((gpio_num_t)BUZZER_PIN_A, 0);
  gpio_set_level((gpio_num_t)BUZZER_PIN_B, 0);
  buzzerState = false;
}

// Simple blocking beep helper (starts tone, waits, stops tone)
static void beep(uint32_t freqHz, uint32_t durationMs)
{
  if (freqHz == 0 || durationMs == 0)
    return;
  startTone(freqHz);
  delay(durationMs);
  stopTone();
}