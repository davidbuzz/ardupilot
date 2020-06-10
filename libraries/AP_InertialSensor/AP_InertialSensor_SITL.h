#pragma once

#include <SITL/SITL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

// simulated sensor rates in Hz. This matches a pixhawk1
const uint16_t INS_SITL_SENSOR_A[] = { 1000, 1000 };
const uint16_t INS_SITL_SENSOR_B[] = { 760, 800 };

class AP_InertialSensor_SITL : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_SITL(AP_InertialSensor &imu, const uint16_t sample_rates[]);

    /* update accel and gyro state */
    bool update() override;
    void start() override;

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu, const uint16_t sample_rates[]);

    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {


       // ar & BOOST_SERIALIZATION_NVP(gyro_sample_hz);//const
       // ar & BOOST_SERIALIZATION_NVP(accel_sample_hz);//const
        ar & BOOST_SERIALIZATION_NVP(gyro_instance);
        ar & BOOST_SERIALIZATION_NVP(accel_instance);
        ar & BOOST_SERIALIZATION_NVP(next_gyro_sample);
        ar & BOOST_SERIALIZATION_NVP(next_accel_sample);
        ar & BOOST_SERIALIZATION_NVP(gyro_time);
        ar & BOOST_SERIALIZATION_NVP(accel_time);
        ar & BOOST_SERIALIZATION_NVP(gyro_motor_phase);
        ar & BOOST_SERIALIZATION_NVP(accel_motor_phase);
        ar & BOOST_SERIALIZATION_NVP(bus_id);


    }
private:
    bool init_sensor(void);
    void timer_update();
    float gyro_drift(void);
    void generate_accel();
    void generate_gyro();

    SITL::SITL *sitl;

    const uint16_t gyro_sample_hz;
    const uint16_t accel_sample_hz;

    uint8_t gyro_instance;
    uint8_t accel_instance;
    uint64_t next_gyro_sample;
    uint64_t next_accel_sample;
    float gyro_time;
    float accel_time;
    float gyro_motor_phase[12];
    float accel_motor_phase[12];

    static uint8_t bus_id;
};
#endif // CONFIG_HAL_BOARD
