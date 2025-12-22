#ifndef RADAR_H
#define RADAR_H

#include <cmath>
#include <cstdint>

struct Radar
{
    double lat[8];
    double lng[8];
    int pointCount;
    int speedLimitKmh;
    double centroidLat;
    double centroidLng;
};

struct RadarAlertResult
{
    bool inZone = false;
    int severity = 0; // 0 = none, 1 = caution, 2 = over limit
    unsigned long intervalMs = 0;
    unsigned long durationMs = 0;
    uint32_t freqHz = 0;
};

class RadarAlertManager
{
public:
    RadarAlertManager();
    // update with position and speed; returns alert info
    RadarAlertResult update(double lat, double lng, double speedKmh);

private:
    bool prevInZone = false;
};

#endif // RADAR_H
