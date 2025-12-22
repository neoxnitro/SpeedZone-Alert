// Driver helper for u-blox GPS (GPS6MV2)
#pragma once
#include <Arduino.h>
#include "driver/uart.h"

// Expose GPS UART port defined in main.cpp
extern uart_port_t GPS_UART;

// Configure GPS update rate to 5Hz (UBX-CFG-RATE)
void sendUbxCfgRate5Hz();

// Configure GPS port to 115200 baud (UBX-CFG-PRT)
void sendUbxCfgPrt115200();
