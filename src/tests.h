#ifndef TESTS_H
#define TESTS_H

#include <Arduino.h>
#include <stdint.h>

// Declarations for functions/vars provided by main.cpp used by tests
void updateRadarAlerts(double lat, double lng, double speedKmh, int sats = -1);
void showMessage(const char *msg, int y, uint16_t color, uint8_t size);
extern int h;

// Test API
void tests_init();
void test_app();
void test_start();
void test_stop();

#endif // TESTS_H
