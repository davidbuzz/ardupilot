#include "AP_Vehicle.h"

#if AP_VEHICLE_ENABLED

#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

void AP_Vehicle::load_parameters(AP_Int16 &format_version, const uint16_t expected_format_version)
{
    hal.console->printf("AP_Vehicle::load_parameters start\n");
    if (!format_version.load() ||
        format_version != expected_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM... actual:%d expected:%d \n", format_version, expected_format_version);
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        format_version.set_and_save(expected_format_version);
        hal.console->printf("done.\n");
    }
    format_version.set_default(expected_format_version);

    hal.console->printf("AP_Vehicle::load_parameters end\n");

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}

#endif  // AP_VEHICLE_ENABLED
