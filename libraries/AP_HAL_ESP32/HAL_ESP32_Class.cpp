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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "HAL_ESP32_Class.h"
#include "Scheduler.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "UARTDriver.h"
#include "WiFiDriver.h"
#include "WiFiUdpDriver.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Storage.h"
#include "AnalogIn.h"
#include "Util.h"

// // #if HAL_ESP32_WIFI == 1
//...
// // #elif HAL_ESP32_WIFI == 2
//...
// // #endif


// cons = uartADriver(0)
#define HAL_UARTA_DRIVER ESP32::UARTDriver    uartADriver(0)
// gps
#define HAL_UARTB_DRIVER ESP32::UARTDriver    uartBDriver(1)
// telem:
#define HAL_UARTC_DRIVER ESP32::UARTDriver    uartCDriver(2)
//tcp, client should connect to 192.168.4.1 port 5760:
#define HAL_UARTD_DRIVER ESP32::WiFiDriver    uartDDriver(3) 
//udp, client should connect as UDPCL to 192.168.4.1 port 14550
//#define HAL_UARTD_DRIVER ESP32::WiFiUdpDriver uartDDriver(3) 


//telem on other uart, if u have one
#define HAL_UARTE_DRIVER Empty::UARTDriver    uartEDriver(4) 
//unused
#define HAL_UARTF_DRIVER Empty::UARTDriver    uartFDriver(5)
#define HAL_UARTG_DRIVER Empty::UARTDriver    uartGDriver(6)
#define HAL_UARTH_DRIVER Empty::UARTDriver    uartHDriver(7)
#define HAL_UARTI_DRIVER Empty::UARTDriver    uartIDriver(8)
#define HAL_UARTJ_DRIVER Empty::UARTDriver    uartJDriver(9)


static HAL_UARTA_DRIVER;
static HAL_UARTB_DRIVER;
static HAL_UARTC_DRIVER;
static HAL_UARTD_DRIVER;
static HAL_UARTE_DRIVER;
static HAL_UARTF_DRIVER;
static HAL_UARTG_DRIVER;
static HAL_UARTH_DRIVER;
static HAL_UARTI_DRIVER;
static HAL_UARTJ_DRIVER;


static Empty::DSP dspDriver;

static ESP32::I2CDeviceManager i2cDeviceManager;
static ESP32::SPIDeviceManager spiDeviceManager;
#ifndef HAL_DISABLE_ADC_DRIVER
static ESP32::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif
static ESP32::Storage storageDriver;
static Empty::GPIO gpioDriver;
static ESP32::RCOutput rcoutDriver;
static ESP32::RCInput rcinDriver;
static ESP32::Scheduler schedulerInstance;
static ESP32::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

// the main entry point of 'main' is hal.run().
extern const AP_HAL::HAL& hal;

HAL_ESP32::HAL_ESP32() :
    AP_HAL::HAL(
        &uartADriver, //Console/mavlink
        &uartBDriver, //GPS 1
        &uartCDriver, //Telem 1
        &uartDDriver, //Telem 2
        &uartEDriver, //GPS 2
        &uartFDriver, //Extra 1
        &uartGDriver, //Extra 2
        &uartHDriver, //Extra 3
        &uartIDriver, //Extra 4
        &uartJDriver, //Extra 5
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
        &dspDriver,
        nullptr
    )
{}

// the main entry point of 'main' is hal.run()... where we basically just run the scheduler's init()..
void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    ((ESP32::Scheduler *)hal.scheduler)->set_callbacks(callbacks);
    hal.scheduler->init();
}

void AP_HAL::init()
{
}

