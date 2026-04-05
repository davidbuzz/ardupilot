/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  DPS280 barometer driver
 */

#include "AP_Baro_DPS280.h"

#if AP_BARO_DPS280_ENABLED

#include <stdio.h>
#include <AP_Math/definitions.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

#define DPS280_REG_PRESS  0x00
#define DPS280_REG_TEMP   0x03
#define DPS280_REG_PCONF  0x06
#define DPS280_REG_TCONF  0x07
#define DPS280_REG_MCONF  0x08
#define DPS280_REG_CREG   0x09
#define DPS280_REG_ISTS   0x0A
#define DPS280_REG_FSTS   0x0B
#define DPS280_REG_RESET  0x0C
#define DPS280_REG_PID    0x0D
#define DPS280_REG_COEF   0x10
#define DPS280_REG_CSRC   0x28

#define DPS280_WHOAMI 0x10

#define TEMPERATURE_LIMIT_C 120

AP_Baro_DPS280::AP_Baro_DPS280(AP_Baro &baro, AP_HAL::Device &_dev)
    : AP_Baro_Backend(baro)
    , dev(&_dev)
    , _dbg_calls(0), _dbg_i2c_fail(0), _dbg_not_ready(0), _dbg_success(0), _dbg_last_report_ms(0)
{
}

AP_Baro_Backend *AP_Baro_DPS280::probe(AP_Baro &baro,
                                       AP_HAL::Device &_dev,
                                       bool _is_dps310)
{
    AP_Baro_DPS280 *sensor = NEW_NOTHROW AP_Baro_DPS280(baro, _dev);
    if (sensor) {
        sensor->is_dps310 = _is_dps310;
    }
    if (!sensor || !sensor->init(_is_dps310)) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

AP_Baro_Backend *AP_Baro_DPS310::probe(AP_Baro &baro, AP_HAL::Device &_dev)
{
    // same as DPS280 but with is_dps310 set for temperature fix
    return AP_Baro_DPS280::probe(baro, _dev, true);
}

/*
  handle bit width for 16 bit config registers
 */
void AP_Baro_DPS280::fix_config_bits16(int16_t &v, uint8_t bits) const
{
    if (v > int16_t((1U<<(bits-1))-1)) {
        v = v - (1U<<bits);
    }
}

/*
  handle bit width for 32 bit config registers
 */
void AP_Baro_DPS280::fix_config_bits32(int32_t &v, uint8_t bits) const
{
    if (v > int32_t((1U<<(bits-1))-1)) {
        v = v - (1U<<bits);
    }
}

/*
  read calibration data
 */
bool AP_Baro_DPS280::read_calibration(void)
{
    uint8_t buf[18];

    if (!dev->read_registers(DPS280_REG_COEF, buf, 18)) {
        return false;
    }

    calibration.C0  = (buf[0] << 4) + ((buf[1] >>4) & 0x0F);
    calibration.C1  = (buf[2] + ((buf[1] & 0x0F)<<8));
    calibration.C00 = ((buf[4]<<4) + (buf[3]<<12)) + ((buf[5]>>4) & 0x0F);
    calibration.C10 = ((buf[5] & 0x0F)<<16) + buf[7] + (buf[6]<<8);
    calibration.C01 = (buf[9] + (buf[8]<<8));
    calibration.C11 = (buf[11] + (buf[10]<<8));
    calibration.C20 = (buf[13] + (buf[12]<<8));
    calibration.C21 = (buf[15] + (buf[14]<<8));
    calibration.C30 = (buf[17] + (buf[16]<<8));

    fix_config_bits16(calibration.C0, 12);
    fix_config_bits16(calibration.C1, 12);
    fix_config_bits32(calibration.C00, 20);
    fix_config_bits32(calibration.C10, 20);
    fix_config_bits16(calibration.C01, 16);
    fix_config_bits16(calibration.C11, 16);
    fix_config_bits16(calibration.C20, 16);
    fix_config_bits16(calibration.C21, 16);
    fix_config_bits16(calibration.C30, 16);

    /* get calibration source */
    if (!dev->read_registers(DPS280_REG_CSRC, &calibration.temp_source, 1)) {
        return false;
    }
    calibration.temp_source &= 0x80;

    return true;
}

void AP_Baro_DPS280::set_config_registers(void)
{
    dev->write_register(DPS280_REG_CREG, 0x0C, true); // shift for 16x oversampling
    dev->write_register(DPS280_REG_PCONF, 0x54, true); // 32 Hz, 16x oversample
    dev->write_register(DPS280_REG_TCONF, 0x54 | calibration.temp_source, true); // 32 Hz, 16x oversample
    dev->write_register(DPS280_REG_MCONF, 0x07); // continuous temp and pressure.

    if (is_dps310) {
        // work around broken temperature handling on some sensors
        // using undocumented register writes
        // see https://github.com/infineon/DPS310-Pressure-Sensor/blob/dps310/src/DpsClass.cpp#L442
        dev->write_register(0x0E, 0xA5);
        dev->write_register(0x0F, 0x96);
        dev->write_register(0x62, 0x02);
        dev->write_register(0x0E, 0x00);
        dev->write_register(0x0F, 0x00);
    }
}

bool AP_Baro_DPS280::init(bool _is_dps310)
{
    if (!dev) {
        return false;
    }
    dev->get_semaphore()->take_blocking();

    // setup to allow reads on SPI
    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // the DPS310 can get into a state on boot where the whoami is not
    // read correctly at startup. Toggling the CS line gets its out of
    // this state
    dev->set_chip_select(true);
    dev->set_chip_select(false);

    uint8_t whoami=0;
    bool ok_read = dev->read_registers(DPS280_REG_PID, &whoami, 1);
    DEV_PRINTF("DPS280 init: read=%d whoami=0x%02x (expect 0x%02x)\n",
               (int)ok_read, (unsigned)whoami, (unsigned)DPS280_WHOAMI);
    if (!ok_read || whoami != DPS280_WHOAMI) {
        dev->get_semaphore()->give();
        return false;
    }

    if (!read_calibration()) {
        DEV_PRINTF("DPS280 init: read_calibration failed\n");
        dev->get_semaphore()->give();
        return false;
    }

    dev->setup_checked_registers(4, 20);

    set_config_registers();

    instance = _frontend.register_sensor();
    if(_is_dps310) {
	    dev->set_device_type(DEVTYPE_BARO_DPS310);
    } else {
	    dev->set_device_type(DEVTYPE_BARO_DPS280);
    }
    set_bus_id(instance, dev->get_bus_id());
    
    dev->get_semaphore()->give();

    // poll at 32Hz to match configured sensor output data rate
    dev->register_periodic_callback((1000 / 32) * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_DPS280::timer, void));

    return true;
}

/*
  calculate corrected pressure and temperature
 */
void AP_Baro_DPS280::calculate_PT(int32_t UT, int32_t UP, float &pressure, float &temperature)
{
    const struct dps280_cal &cal = calibration;
    // scaling for 16x oversampling
    const float scaling_16 = 1.0f/253952;

    float temp_scaled;
    float press_scaled;

    temp_scaled = float(UT) * scaling_16;
    temperature = cal.C0 * 0.5f + cal.C1 * temp_scaled;

    press_scaled = float(UP) * scaling_16;

    pressure = cal.C00;
    pressure += press_scaled * (cal.C10 + press_scaled * (cal.C20 + press_scaled * cal.C30));
    pressure += temp_scaled * cal.C01;
    pressure += temp_scaled * press_scaled * (cal.C11 + press_scaled * cal.C21);
}

/*
  check health and possibly reset
 */
void AP_Baro_DPS280::check_health(void)
{
    dev->check_next_register();

    if (fabsf(last_temperature) > TEMPERATURE_LIMIT_C) {
        err_count++;
    }
    if (err_count > 16) {
        err_count = 0;
        dev->write_register(DPS280_REG_RESET, 0x09);
        set_config_registers();
        pending_reset = true;
    }
}

//  accumulate a new sensor reading
void AP_Baro_DPS280::timer(void)
{
    uint8_t buf[6];
    uint8_t ready;

    // Increment diagnostic counter for this timer call
    _dbg_calls++;

    if (pending_reset) {
        // After a soft reset the DPS310 must reload its calibration
        // coefficients before it will accept a new measurement-mode write.
        // COEF_RDY (bit7) and SENSOR_RDY (bit6) must both be 1 first.
        uint8_t mconf = 0;
        if (!dev->read_registers(DPS280_REG_MCONF, &mconf, 1)) {
            return;  // I2C fail during recovery - try again next call
        }
        if ((mconf & 0xC0) != 0xC0) {
            // Chip still re-initialising after soft reset - wait
            return;
        }
        pending_reset = false;
        set_config_registers();
        return;
    }

    if (!dev->read_registers(DPS280_REG_MCONF, &ready, 1)) {
        // I2C read of MCONF completely failed (timeout, NACK, etc.)
        _dbg_i2c_fail++;
        err_count++;
        check_health();
        return;
    }

    if (!(ready & (1U<<4))) {
        // Pressure measurement result not yet available.
        // This is normal in continuous mode between measurements and must
        // NOT be counted as an error - doing so caused a reset-loop.
        _dbg_not_ready++;
        return;
    }

    if (!dev->read_registers(DPS280_REG_PRESS, buf, 3) ||
        !dev->read_registers(DPS280_REG_TEMP, &buf[3], 3)) {
        // PRS_RDY was set but the data read failed - genuine bus error
        _dbg_not_ready++;
        err_count++;
        check_health();
        return;
    }
    _dbg_success++;

    int32_t press = (buf[2]) + (buf[1]<<8) + (buf[0]<<16);
    int32_t temp  = (buf[5]) + (buf[4]<<8) + (buf[3]<<16);
    fix_config_bits32(press, 24);
    fix_config_bits32(temp, 24);

    float pressure, temperature;

    calculate_PT(temp, press, pressure, temperature);

    last_temperature = temperature;

    if (!pressure_ok(pressure)) {
        return;
    }

    check_health();

    if (fabsf(last_temperature) <= TEMPERATURE_LIMIT_C) {
        err_count = 0;
    }
    
    WITH_SEMAPHORE(_sem);

    pressure_sum += pressure;
    temperature_sum += temperature;
    count++;
}

// transfer data to the frontend
void AP_Baro_DPS280::update(void)
{
    float dbg_pressure = 0.0f;
    float dbg_temperature = last_temperature;

    // Diagnostic: report timer counters every ~10s from main thread (safe for GCS_SEND_TEXT).
    // This helps diagnose why DPS310 stays unhealthy after probe succeeds.
    uint32_t now_ms = AP_HAL::millis();

    // Snapshot one representative sensor sample for debug output.
    WITH_SEMAPHORE(_sem);
    if (count > 0) {
        dbg_pressure = pressure_sum / count;
        dbg_temperature = temperature_sum / count;
    }

    if (now_ms - _dbg_last_report_ms > 10000) {
        _dbg_last_report_ms = now_ms;
        // Include live pressure and temperature so each debug report carries
        // sensor values, not only transport/ready counters.
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                      "DPS310 dbg: calls=%u i2c_fail=%u notready=%u ok=%u p=%.2fPa t=%.2fC",
                      (unsigned)_dbg_calls, (unsigned)_dbg_i2c_fail,
                      (unsigned)_dbg_not_ready, (unsigned)_dbg_success,
                      (double)dbg_pressure,
                      (double)dbg_temperature);
    }

    if (count == 0) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    _copy_to_frontend(instance, pressure_sum/count, temperature_sum/count);
    pressure_sum = 0;
    temperature_sum = 0;
    count=0;
}

#endif  // AP_BARO_DPS280_ENABLED
