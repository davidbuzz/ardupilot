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

static ESP32::UARTDriver uartADriver(0);// cons, real uarts need a idx to attach to, wifi doesn't
static Empty::UARTDriver uartBDriver; //egads, uartBDriver is the port that ardupilot probes for a GPS on.
static ESP32::WiFiDriver uartCDriver; //lets try for tcp/mavlink on uart 'C'. tcp, client should connect to 192.168.4.1 port 5760

//static ESP32::WiFiUdpDriver uartCDriver; //udp
//static Empty::UARTDriver uartCDriver; // empty doesn't need an idx either
static Empty::UARTDriver uartDDriver;
static Empty::UARTDriver uartEDriver;
static Empty::UARTDriver uartFDriver;
static Empty::UARTDriver uartGDriver;
static Empty::UARTDriver uartHDriver;
static Empty::UARTDriver uartIDriver;
static Empty::UARTDriver uartJDriver;

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

void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    ((ESP32::Scheduler *)hal.scheduler)->set_callbacks(callbacks);
    hal.scheduler->init();
    hal.console->printf("\n%s:%d scheduler init() done\n", __PRETTY_FUNCTION__, __LINE__);
}

void AP_HAL::init()
{
}

