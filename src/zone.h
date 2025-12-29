#pragma once
#include <stdint.h>

// Initialize with exactly 4 points (lat, lon)
void zone_init(const double pts[4][2]);
// Call with current position and optionally epoch/time validity.
// If timeValid==true pass epoch (seconds since epoch), else epoch may be 0.
// Returns: 'E' for entry recorded, 'X' for exit recorded, 0 for nothing.
char zone_process(double lat, double lng, bool timeValid, uint32_t epoch);
