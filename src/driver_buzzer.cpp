#include "driver_buzzer.h"
#include "driver/gpio.h"

static hw_timer_t *buzzerTimer = NULL;
static volatile bool buzzerState = false;
static int buzzerPinA = -1;
static int buzzerPinB = -1;

// Timer ISR toggles the two buzzer pins in opposite phase to increase swing.
void IRAM_ATTR onBuzzerTimer()
{
    buzzerState = !buzzerState;
    gpio_set_level((gpio_num_t)buzzerPinA, buzzerState ? 1 : 0);
    gpio_set_level((gpio_num_t)buzzerPinB, buzzerState ? 0 : 1);
}

void buzzerInit(int pinA, int pinB)
{
    buzzerPinA = pinA;
    buzzerPinB = pinB;
    pinMode(buzzerPinA, OUTPUT);
    pinMode(buzzerPinB, OUTPUT);
    digitalWrite(buzzerPinA, LOW);
    digitalWrite(buzzerPinB, LOW);
}

void startTone(uint32_t freqHz)
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

void stopTone()
{
    if (buzzerTimer)
    {
        timerAlarmDisable(buzzerTimer);
    }
    gpio_set_level((gpio_num_t)buzzerPinA, 0);
    gpio_set_level((gpio_num_t)buzzerPinB, 0);
    buzzerState = false;
}

void beep(uint32_t freqHz, uint32_t durationMs)
{
    if (freqHz == 0 || durationMs == 0)
        return;
    startTone(freqHz);
    delay(durationMs);
    stopTone();
}
