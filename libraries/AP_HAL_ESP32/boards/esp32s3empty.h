/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifdef BUILT_WITH_CMAKE
#include "config/sdkconfig.h"
#endif

#include <hal/gpio_types.h>
#include <hal/i2c_types.h>
#include <driver/uart.h>

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3EMPTY
// make sensor selection clearer

//- these are missing from esp-idf......will not be needed later
#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4

#define HAL_ESP32_BOARD_NAME "esp32s3empty"

#define HAL_ESP32_RMT_RX_PIN_NUMBER GPIO_NUM_14

// no sensors
#define HAL_INS_DEFAULT HAL_INS_NONE

#define HAL_BARO_ALLOW_INIT_NO_BARO 1

#define AP_COMPASS_ENABLE_DEFAULT 0
#define ALLOW_ARM_NO_COMPASS
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

// no ADC
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

// 2 use udp, 1 use tcp...  for udp,client needs to connect as UDPCL in missionplanner etc to 192.168.4.1 port 14550
#define HAL_ESP32_WIFI 2

// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

#define WIFI_SSID "ardupilot123"
#define WIFI_PWD "ardupilot123"

//RCOUT which pins are used?

#define HAL_ESP32_RCOUT { GPIO_NUM_11,GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_8, GPIO_NUM_7, GPIO_NUM_6 }

// SPI BUS setup, including gpio, dma, etc
// note... we use 'vspi' for the bmp280 and mpu9250
#define HAL_ESP32_SPI_BUSES {}

// SPI per-device setup, including speeds, etc.
#define HAL_ESP32_SPI_DEVICES {}

//I2C bus list
#define HAL_ESP32_I2C_BUSES {.port=I2C_NUM_0, .sda=GPIO_NUM_13, .scl=GPIO_NUM_14, .speed=400*KHZ, .internal=true, .soft=true}

// rcin on what pin?
#define HAL_ESP32_RCIN GPIO_NUM_14

#define HAL_NMEA_OUTPUT_ENABLED 0


//HARDWARE UARTS
#define HAL_ESP32_UART_DEVICES   {.port=UART_NUM_0, .rx=GPIO_NUM_44, .tx=GPIO_NUM_43 },{.port=UART_NUM_1, .rx=GPIO_NUM_17, .tx=GPIO_NUM_18 }

#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 0

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

#define HAL_LOGGING_BACKENDS_DEFAULT 1

#define AP_RCPROTOCOL_ENABLED 0

#define AP_FILESYSTEM_ESP32_ENABLED 1
#define AP_SCRIPTING_ENABLED 0
//#define HAL_USE_EMPTY_STORAGE 1


// the below list is basically from chibios's minimise_common.inc and represents a "hal minimise" for the ESP32

//  this include file is used to remove features which will never be
//   wanted on any low-flash board in our standard builds.  It is to be
//   included by other minimize_*.inc files and not generally used
//   otherwise.

//  disable emitting nice strings when activating RC channel aux functions:
#define AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED 0

// low-end boards aren't expected to be used in environments where
// things like satellite communications are required:
#define HAL_HIGH_LATENCY2_ENABLED 0

// Gripper isn't a vital feature for smaller boards
#define AP_GRIPPER_ENABLED 0

// Sprayer isn't a vital feature for smaller boards
#define HAL_SPRAYER_ENABLED 0

// disable use of onboard FFT library:
#define HAL_GYROFFT_ENABLED 0

// several notify backends are rare and not supported by default on smaller boards:
#define AP_NOTIFY_NCP5623_ENABLED 0

// HOTT telemetry is quite rare, so we don't include it on smaller boards
#define HAL_HOTT_TELEM_ENABLED 0

// smaller boards lose some GPS support
#define AP_GPS_BACKEND_DEFAULT_ENABLED 0
#define AP_GPS_UBLOX_ENABLED 1
#define AP_GPS_DRONECAN_ENABLED HAL_ENABLE_DRONECAN_DRIVERS
#undef HAL_MSP_GPS_ENABLED
#define HAL_MSP_GPS_ENABLED HAL_MSP_SENSORS_ENABLED

// no moving baseline support:
#define GPS_MOVING_BASELINE 0

// No LTM telemetry on minimized boards:
#define AP_LTM_TELEM_ENABLED 0

// various structures increase the flash size when using >16 servos:
#define NUM_SERVO_CHANNELS 16

// no Winch if minimized:
#define AP_WINCH_ENABLED 0

// prune out some odd camera backends:
#define AP_CAMERA_BACKEND_DEFAULT_ENABLED 0
#define AP_CAMERA_RELAY_ENABLED AP_CAMERA_ENABLED
#define AP_CAMERA_SERVO_ENABLED AP_CAMERA_ENABLED

// no SLCAN on these boards (use can-over-mavlink if required)
#define AP_CAN_SLCAN_ENABLED 0

// no PiccoloCAN:
#define HAL_PICCOLO_CAN_ENABLE 0

// no beacon support on minimized boards:
#define AP_BEACON_ENABLED 0

// restricted battery backends:
#define AP_BATTERY_BACKEND_DEFAULT_ENABLED 0
#define AP_BATTERY_ANALOG_ENABLED 1
#define AP_BATTERY_ESC_ENABLED HAL_WITH_ESC_TELEM
#define AP_BATTERY_WATT_MAX_ENABLED 0
#define AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED HAL_ENABLE_DRONECAN_DRIVERS
#define AP_BATTERY_SUM_ENABLED 1
#define AP_BATTERY_SYNTHETIC_CURRENT_ENABLED 1
#define AP_BATTERY_SMBUS_ENABLED 1
#define AP_BATTERY_SMBUS_GENERIC_ENABLED AP_BATTERY_SMBUS_ENABLED
#define AP_BATTERY_SMBUS_NEODESIGN_ENABLED AP_BATTERY_SMBUS_ENABLED
#define AP_BATTERY_SMBUS_SUI_ENABLED AP_BATTERY_SMBUS_ENABLED
#define AP_BATTERY_SMBUS_MAXELL_ENABLED AP_BATTERY_SMBUS_ENABLED
#define AP_BATTERY_SMBUS_ROTOYE_ENABLED AP_BATTERY_SMBUS_ENABLED

// don't probe for external Barometers:
#define AP_BARO_PROBE_EXTERNAL_I2C_BUSES 0

// no wind compensation code:
#define HAL_BARO_WIND_COMP_ENABLED 0

// no mounts:
#define HAL_MOUNT_ENABLED 0

// no generator:
#define HAL_GENERATOR_ENABLED 0

// no NMEA output:
#define HAL_NMEA_OUTPUT_ENABLED 0

// no Notify Display support:
#define HAL_DISPLAY_ENABLED 0

// remove support for killing IMUs at runtime - a developer feature:
#define AP_INERTIALSENSOR_KILL_IMU_ENABLED 0

// shortened names in @SYS/taskinfo.txt
#define AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED 0

// Plane-specific defines; these defines are only used in the Plane
//  directory, but are seen across the entire codebase:
#define OFFBOARD_GUIDED 0
#define QAUTOTUNE_ENABLED 0

// Copter-specific defines; these defines are only used in the Copter
//  directory, but are seen across the entire codebase:
#define MODE_FLOWHOLD_ENABLED 0
#define MODE_ZIGZAG_ENABLED 0
#define AC_NAV_GUIDED 0
#define AP_OAPATHPLANNER_ENABLED 0
#define MODE_FOLLOW_ENABLED 0
#define MODE_GUIDED_NOGPS_ENABLED 0
#define MODE_SYSTEMID_ENABLED 0
#define WEATHERVANE_ENABLED 0
#define MODE_AUTOROTATE_ENABLED 0

// don't send RELAY_STATUS messages:
#define AP_MAVLINK_MSG_RELAY_STATUS_ENABLED 0

//#fewer airspeed sensors
// #define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0
// #define AP_AIRSPEED_MS4525_ENABLED 1
// #define AP_AIRSPEED_ASP5033_ENABLED 1
// #define AP_AIRSPEED_MS5525_ENABLED 1
// #define AP_AIRSPEED_SDP3X_ENABLED 1
// #define AP_AIRSPEED_NMEA_ENABLED 1  # additional checks for vehicle type in .cpp
// #define AP_AIRSPEED_DRONECAN_ENABLED HAL_ENABLE_DRONECAN_DRIVERS

// don't need payload place mission items on these boards, it's very niche:
#define AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED 0
// don't need the payload place flight behaviour either:
#define AC_PAYLOAD_PLACE_ENABLED 0
