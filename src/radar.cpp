#include "radar.h"
#include <cstring>
#include <cmath>

static const Radar radars[] = {
    // radar 0 (to camela)
    {{28.070931674444864, 28.070733016425383, 28.070082389279616, 28.070414850616544},
     {-16.704965032804946, -16.70501371221718, -16.701937229252348, -16.70191839016962},
     4,
     70,
     28.070572133618313,
     -16.70423854319459},
    // radar 1 (chafira)
    {{28.05809872948754, 28.05821059877128, 28.056935977954055, 28.056800692550237},
     {-16.582544853034527, -16.58261118527233, -16.586532095974285, -16.58648050423377},
     4,
     120,
     28.057944079982274,
     -16.583244252771262},
    // radar 2 (las americas exit)
    {{28.065212620910614, 28.065404455437093, 28.06475137083114, 28.06451492213801},
     {-16.71731215868648, -16.717261601025573, -16.71408477371745, -16.71430385691472},
     4,
     60,
     28.065071387502737,
     -16.71600184039197},
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
