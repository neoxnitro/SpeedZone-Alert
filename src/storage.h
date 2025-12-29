#pragma once
#include <stdint.h>

bool storage_init();
bool storage_append_event(char evType, uint32_t epoch);
// returns epoch of last recorded entry event or 0 if none
uint32_t storage_get_last_entry_epoch();
