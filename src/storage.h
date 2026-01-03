#pragma once
#include <stdint.h>

bool storage_init();
bool storage_append_event(char evType, uint32_t epoch);
// returns epoch of last recorded entry event or 0 if none
uint32_t storage_get_last_entry_epoch();
// Print file contents to Serial with simple formatted borders
void storage_print_file(const char *path);
// Convenience: print storage-related files (log + last-entry)
void storage_dump_state();
