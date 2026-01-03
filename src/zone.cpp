#include "zone.h"
#include "storage.h"
#include <Arduino.h>
#include <time.h>

// Haversine distance in meters (local copy to avoid cross-file dependency)
static double haversineMeters(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0; // earth radius meters
    const double PI_VAL = 3.14159265358979323846;
    double dLat = (lat2 - lat1) * PI_VAL / 180.0;
    double dLon = (lon2 - lon1) * PI_VAL / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * PI_VAL / 180.0) * cos(lat2 * PI_VAL / 180.0) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

static double zone_pts[4][2];
static uint32_t lastEntryEpoch = 0; // epoch of last recorded entry

// simple point-in-polygon for convex 4-point polygon using ray casting
static bool point_in_poly(double lat, double lng)
{
    int i, j, c = 0;
    for (i = 0, j = 3; i < 4; j = i++)
    {
        double yi = zone_pts[i][0], xi = zone_pts[i][1];
        double yj = zone_pts[j][0], xj = zone_pts[j][1];
        bool intersect = ((yi > lat) != (yj > lat)) &&
                         (lng < (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi);
        if (intersect)
            c = !c;
    }
    return c != 0;
}

void zone_init(const double pts[4][2])
{
    for (int i = 0; i < 4; ++i)
    {
        zone_pts[i][0] = pts[i][0];
        zone_pts[i][1] = pts[i][1];
    }
    lastEntryEpoch = storage_get_last_entry_epoch();
}

char zone_process(double lat, double lng, bool timeValid, uint32_t epoch)
{
    // First check distance to first zone point to avoid expensive polygon test
    double d0 = haversineMeters(lat, lng, zone_pts[0][0], zone_pts[0][1]);
    bool inside = false;
    if (d0 <= 200.0)
    {
        inside = point_in_poly(lat, lng);
    }

    // Entry
    if (inside && timeValid && epoch != 0 && lastEntryEpoch == 0)
    {
        if (storage_append_event('E', epoch))
        {
            lastEntryEpoch = epoch;
            Serial.printf("[zone] Entry recorded epoch=%u\n", (unsigned)epoch);
            return 'E';
        }
    }
    // Exit
    if (!inside && timeValid && epoch != 0 && lastEntryEpoch != 0 && (epoch - lastEntryEpoch) >= 600)
    {
        if (storage_append_event('X', epoch))
        {
            lastEntryEpoch = 0;
            Serial.printf("[zone] Auto-exit recorded epoch=%u (clearing lastEntryEpoch)\n", (unsigned)epoch);
            return 'X';
        }
    }
    return '\0';
}
