#include "storage.h"
#include <LittleFS.h>
#include <FS.h>
#include <Arduino.h>

static const char *LOG_PATH = "/zone_log.csv";
static const char *LAST_ENTRY_PATH = "/last_entry.txt";

bool storage_init()
{
    if (!LittleFS.begin())
    {
        Serial.println("[storage] LittleFS mount failed - attempting format (will erase FS)");
        // Try to format the filesystem to recover from corruption
        if (!LittleFS.format())
        {
            Serial.println("[storage] LittleFS format failed");
            return false;
        }
        delay(100);
        if (!LittleFS.begin())
        {
            Serial.println("[storage] LittleFS mount failed after format");
            return false;
        }
        Serial.println("[storage] LittleFS formatted and mounted successfully");
    }
    // ensure file exists
    if (!LittleFS.exists(LOG_PATH))
    {
        File f = LittleFS.open(LOG_PATH, FILE_WRITE);
        if (f)
            f.close();
    }
    // ensure last-entry quick lookup file exists (store single epoch as text)
    if (!LittleFS.exists(LAST_ENTRY_PATH))
    {
        File f = LittleFS.open(LAST_ENTRY_PATH, FILE_WRITE);
        if (f)
        {
            f.print("0\n");
            f.close();
        }
    }

    // If both files are empty (fresh littlefs), write a single default entry
    bool logEmpty = true;
    bool lastEmpty = true;
    String lastEntryStr = "";
    if (LittleFS.exists(LOG_PATH))
    {
        File f = LittleFS.open(LOG_PATH, FILE_READ);
        if (f)
        {
            if (f.size() > 0)
                logEmpty = false;
            f.close();
        }
    }
    if (LittleFS.exists(LAST_ENTRY_PATH))
    {
        File f = LittleFS.open(LAST_ENTRY_PATH, FILE_READ);
        if (f)
        {
            if (f.size() > 0)
            {
                lastEntryStr = f.readStringUntil('\n');
                lastEntryStr.trim();
                // treat "0" as empty/default
                if (lastEntryStr.length() == 0 || lastEntryStr == "0")
                    lastEmpty = true;
                else
                    lastEmpty = false;
            }
            f.close();
        }
    }

    Serial.printf("[storage] logEmpty=%d lastEntry='%s'\n", logEmpty ? 1 : 0, lastEntryStr.c_str());
    /*
        if (logEmpty && lastEmpty)
        {
            const uint32_t defaultEpoch = 1767459089UL;
            File f = LittleFS.open(LOG_PATH, FILE_WRITE);
            if (f)
            {
                char buf[64];
                int n = snprintf(buf, sizeof(buf), "E,%u\n", (unsigned)defaultEpoch);
                if (n > 0)
                    f.write((const uint8_t *)buf, n);
                f.close();
            }
            File f2 = LittleFS.open(LAST_ENTRY_PATH, FILE_WRITE);
            if (f2)
            {
                char eb[32];
                int en = snprintf(eb, sizeof(eb), "%u\n", (unsigned)defaultEpoch);
                if (en > 0)
                    f2.write((const uint8_t *)eb, en);
                f2.close();
            }
            Serial.println("[storage] Wrote default entry to LittleFS");
        }
    */
    return true;
}

bool storage_append_event(char evType, uint32_t epoch)
{
    File f = LittleFS.open(LOG_PATH, FILE_APPEND);
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
        // If this is an entry event, update the quick lookup file so we don't need
        // to scan the full log when querying last entry epoch.
        if (evType == 'E')
        {
            File f2 = LittleFS.open(LAST_ENTRY_PATH, FILE_WRITE);
            if (f2)
            {
                char eb[32];
                int en = snprintf(eb, sizeof(eb), "%u\n", (unsigned)epoch);
                f2.write((const uint8_t *)eb, en);
                f2.close();
            }
        }
        else
        {
            storage_clear_last_entry_epoch();
        }
        return written == (size_t)n;
    }
    f.close();
    return false;
}

uint32_t storage_get_last_entry_epoch()
{
    // Fast path: read small file that stores last entry epoch (written on append)
    if (LittleFS.exists(LAST_ENTRY_PATH))
    {
        File f = LittleFS.open(LAST_ENTRY_PATH, FILE_READ);
        if (f)
        {
            String s = f.readStringUntil('\n');
            f.close();
            s.trim();
            if (s.length() == 0)
                return 0;
            return (uint32_t)strtoul(s.c_str(), NULL, 10);
        }
    }
    // Fallback: no quick lookup file present — return 0 to avoid long blocking scan
    return 0;
}

void storage_clear_last_entry_epoch()
{
    File f = LittleFS.open(LAST_ENTRY_PATH, FILE_WRITE);
    if (f)
    {
        f.print("0\n");
        f.close();
    }
}

// Print a simple dashed border
static void _print_border()
{
    for (int i = 0; i < 56; ++i)
        Serial.print('-');
    Serial.println();
}

void storage_print_file(const char *path)
{
    _print_border();
    Serial.print("| File: ");
    Serial.println(path);
    _print_border();

    if (!LittleFS.exists(path))
    {
        Serial.println("| <missing>");
        _print_border();
        return;
    }

    File f = LittleFS.open(path, FILE_READ);
    if (!f)
    {
        Serial.println("| <open failed>");
        _print_border();
        return;
    }

    while (f.available())
    {
        String line = f.readStringUntil('\n');
        line.trim();
        Serial.print("| ");
        Serial.println(line);
    }
    f.close();
    _print_border();
}

void storage_dump_state()
{
    storage_print_file(LOG_PATH);
    storage_print_file(LAST_ENTRY_PATH);
}
