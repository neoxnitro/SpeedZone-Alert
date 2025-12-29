#include "zone.h"
#include "storage.h"
#include <Arduino.h>
#include <time.h>

static double zone_pts[4][2];
static bool inZone = false;
static uint32_t lastEntryEpoch = 0; // epoch of last recorded entry
static int lastExitDay = -1;        // day-of-year of last exit to limit one exit per day
static bool pendingEntryNoTime = false;
static bool pendingExitNoTime = false;

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
    inZone = false;
    lastEntryEpoch = storage_get_last_entry_epoch();
    pendingEntryNoTime = false;
    pendingExitNoTime = false;
}

static int day_of_year_from_epoch(uint32_t epoch)
{
    time_t t = (time_t)epoch;
    struct tm tm;
    gmtime_r(&t, &tm);
    return tm.tm_yday;
}

char zone_process(double lat, double lng, bool timeValid, uint32_t epoch)
{
    bool inside = point_in_poly(lat, lng);
    // Transition: outside -> inside => entry
    if (inside && !inZone)
    {
        // entered polygon
        if (timeValid && epoch != 0)
        {
            // Suppress if last entry was < 1h ago
            if (lastEntryEpoch == 0 || (epoch - lastEntryEpoch) >= 3600)
            {
                if (storage_append_event('E', epoch))
                {
                    lastEntryEpoch = epoch;
                    inZone = true;
                    pendingEntryNoTime = false;
                    pendingExitNoTime = false;
                    Serial.printf("[zone] Entry recorded epoch=%u\n", (unsigned)epoch);
                    return 'E';
                }
            }
            else
            {
                // suppressed due to <1h
                inZone = true;
                pendingEntryNoTime = false;
                return 0;
            }
        }
        else
        {
            // no valid time yet: mark pending and wait for time
            pendingEntryNoTime = true;
            inZone = true;
            Serial.println("[zone] Entry detected but time invalid - pending");
            return 0;
        }
    }

    // Transition: inside -> outside => exit
    if (!inside && inZone)
    {
        if (timeValid && epoch != 0)
        {
            // if lastEntryEpoch exists and less than 1h ago, suppress
            if (lastEntryEpoch != 0 && (epoch - lastEntryEpoch) < 3600)
            {
                // suppress exit because entry was recent
                inZone = false; // still update state though
                pendingExitNoTime = false;
                Serial.println("[zone] Exit suppressed (<1h since entry)");
                return 0;
            }

            // Only one exit per day
            int doy = day_of_year_from_epoch(epoch);
            if (lastExitDay == doy)
            {
                inZone = false;
                Serial.println("[zone] Exit suppressed (already recorded today)");
                return 0;
            }

            if (storage_append_event('X', epoch))
            {
                lastExitDay = doy;
                inZone = false;
                pendingExitNoTime = false;
                Serial.printf("[zone] Exit recorded epoch=%u\n", (unsigned)epoch);
                return 'X';
            }
            return 0;
        }
        else
        {
            pendingExitNoTime = true;
            inZone = false;
            Serial.println("[zone] Exit detected but time invalid - pending");
            return 0;
        }
    }

    // If we have pending entry/no time and time is now valid, record it
    if (pendingEntryNoTime && timeValid && epoch != 0)
    {
        if (lastEntryEpoch == 0 || (epoch - lastEntryEpoch) >= 3600)
        {
            if (storage_append_event('E', epoch))
            {
                lastEntryEpoch = epoch;
                pendingEntryNoTime = false;
                Serial.printf("[zone] Pending entry recorded epoch=%u\n", (unsigned)epoch);
                return 'E';
            }
        }
        else
        {
            pendingEntryNoTime = false;
        }
    }

    // If we have pending exit/no time and time is now valid, record it
    if (pendingExitNoTime && timeValid && epoch != 0)
    {
        int doy = day_of_year_from_epoch(epoch);
        if (lastExitDay != doy)
        {
            if (storage_append_event('X', epoch))
            {
                lastExitDay = doy;
                pendingExitNoTime = false;
                Serial.printf("[zone] Pending exit recorded epoch=%u\n", (unsigned)epoch);
                return 'X';
            }
        }
        else
        {
            pendingExitNoTime = false;
        }
    }

    return 0;
}
