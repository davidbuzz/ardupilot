#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// INDI rate control with online control-effectiveness identification.
// Not yet wired into any vehicle; enabled on SITL so the unit tests build.
#ifndef AP_INDI_ENABLED
#define AP_INDI_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_SITL
#endif
