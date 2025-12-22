#include "radar.h"
#include <cstring>
#include <cmath>

static const Radar radars[] = {
    // radar 0
    {{28.070786, 28.069711, 28.070598},
     {-16.704797, -16.701104, -16.701104},
     3,
     70,
     28.070806175652283,
     -16.704801935638155},
    // radar 0
    {{28.070796708895237, 28.070143499853952, 28.071743351749255},
     {-16.704812664450014, -16.707344669750903, -16.70736617792127},
     3,
     70,
     28.070806175652283,
     -16.704801935638155},
    // radar 1
    {{28.056966306914898, 28.058879736842236, 28.058574564049025},
     {-16.58595846755022, -16.5812286320508, -16.581102657945877},
     3,
     120,
     28.057751686349476,
     -16.583807174456027},
};
static const int RADAR_COUNT = sizeof(radars) / sizeof(radars[0]);

constexpr double PI_VAL = 3.14159265358979323846;

// Haversine distance in meters
static double haversineMeters(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0; // earth radius meters
    double dLat = (lat2 - lat1) * PI_VAL / 180.0;
    double dLon = (lon2 - lon1) * PI_VAL / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * PI_VAL / 180.0) * cos(lat2 * PI_VAL / 180.0) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

// Ray-casting point-in-polygon test (lat/lon treated as planar since zones are small)
static bool pointInPolygon(double lat, double lon, const Radar &r)
{
    bool inside = false;
    for (int i = 0, j = r.pointCount - 1; i < r.pointCount; j = i++)
    {
        double xi = r.lat[i], yi = r.lng[i];
        double xj = r.lat[j], yj = r.lng[j];
        bool intersect = ((yi > lon) != (yj > lon)) && (lat < (xj - xi) * (lon - yi) / (yj - yi + 1e-12) + xi);
        if (intersect)
            inside = !inside;
    }
    return inside;
}

RadarAlertManager::RadarAlertManager() : prevInZone(false) {}

RadarAlertResult RadarAlertManager::update(double lat, double lng, double speedKmh)
{
    RadarAlertResult res;

    bool inAnyZone = false;
    int bestSeverity = 0;

    for (int i = 0; i < RADAR_COUNT; ++i)
    {
        const Radar &r = radars[i];
        double d = haversineMeters(lat, lng, r.centroidLat, r.centroidLng);
        if (d > 700.0)
            continue;
        if (!pointInPolygon(lat, lng, r))
            continue;
        inAnyZone = true;
        if (speedKmh >= r.speedLimitKmh)
        {
            if (bestSeverity < 3)
                bestSeverity = 3;
        }
        else if (speedKmh >= r.speedLimitKmh - 5)
        {
            if (bestSeverity < 2)
                bestSeverity = 2;
        }
        else
        {
            if (bestSeverity < 1)
                bestSeverity = 1;
        }
    }

    res.inZone = inAnyZone;
    res.severity = bestSeverity;

    if (!inAnyZone || bestSeverity == 0)
    {
        return res;
    }

    switch (bestSeverity)
    {
    case 1: // caution
        res.intervalMs = 10000;
        res.durationMs = 150;
        res.freqHz = 1500;
        break;
    case 2: // over limit
        res.intervalMs = 2000;
        res.durationMs = 200;
        res.freqHz = 2000;
        break;
    case 3: // well over limit
        res.intervalMs = 1000;
        res.durationMs = 300;
        res.freqHz = 2500;
        break;

    default:
        break;
    }

    return res;
}
