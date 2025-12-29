#include "storage.h"
#include <SPIFFS.h>
#include <FS.h>
#include <Arduino.h>

static const char *LOG_PATH = "/zone_log.csv";

bool storage_init()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("[storage] SPIFFS mount failed");
        return false;
    }
    // ensure file exists
    if (!SPIFFS.exists(LOG_PATH))
    {
        File f = SPIFFS.open(LOG_PATH, FILE_WRITE);
        if (f)
            f.close();
    }
    return true;
}

bool storage_append_event(char evType, uint32_t epoch)
{
    File f = SPIFFS.open(LOG_PATH, FILE_APPEND);
    if (!f)
    {
        Serial.println("[storage] open append failed");
        return false;
    }
    // Format: E,epoch\n or X,epoch\n
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%c,%u\n", evType, (unsigned)epoch);
    if (n > 0)
    {
        size_t written = f.write((const uint8_t *)buf, n);
        f.close();
        return written == (size_t)n;
    }
    f.close();
    return false;
}

uint32_t storage_get_last_entry_epoch()
{
    if (!SPIFFS.exists(LOG_PATH))
        return 0;
    File f = SPIFFS.open(LOG_PATH, FILE_READ);
    if (!f)
        return 0;
    // Read file backwards to find last line starting with 'E,'
    int64_t pos = f.size() - 1;
    String line = "";
    while (pos >= 0)
    {
        f.seek(pos);
        char c = f.read();
        if (c == '\n' && line.length() > 0)
        {
            // line currently reversed
            line.trim();
            line = String(line.c_str());
            // reverse line
            int L = line.length();
            String rev = "";
            for (int i = L - 1; i >= 0; --i)
                rev += line[i];
            if (rev.length() > 2 && rev.charAt(0) == 'E' && rev.charAt(1) == ',')
            {
                uint32_t epoch = (uint32_t)rev.substring(2).toInt();
                f.close();
                return epoch;
            }
            line = "";
        }
        else if (c != '\r')
        {
            line += c; // building reversed
        }
        pos--;
    }
    // check remaining
    if (line.length() > 0)
    {
        int L = line.length();
        String rev = "";
        for (int i = L - 1; i >= 0; --i)
            rev += line[i];
        if (rev.length() > 2 && rev.charAt(0) == 'E' && rev.charAt(1) == ',')
        {
            uint32_t epoch = (uint32_t)rev.substring(2).toInt();
            f.close();
            return epoch;
        }
    }
    f.close();
    return 0;
}
