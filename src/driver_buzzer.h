// Buzzer driver: non-blocking tones using hw timer
#pragma once
#include <Arduino.h>

// Initialize buzzer pins (pinA, pinB) and prepare timer
void buzzerInit(int pinA, int pinB);

// Start a square wave on the two pins with given frequency (Hz)
void startTone(uint32_t freqHz);
// Stop the square wave
void stopTone();
// Simple blocking beep helper (starts tone, waits, stops tone)
void beep(uint32_t freqHz, uint32_t durationMs);
