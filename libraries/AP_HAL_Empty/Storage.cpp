
#include <string.h>
#include "Storage.h"

using namespace Empty;

extern const AP_HAL::HAL& hal;


Storage::Storage()
{}

void Storage::init()
{}

void Storage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
    static bool warned = false;
    if (!warned) {
        hal.console->printf("AP_HAL_EMPTY Storage::read_block - WARNING! Your storage isn't persistent\n");
        warned = true;
    }
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
    static bool warned = false;
    if (!warned) {
        hal.console->printf("AP_HAL_EMPTY Storage::write_block - WARNING! Your storage isn't persistent\n");
        warned = true;
    }   
}

