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
  simple electric motor simulation class
*/

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/Serialize.h>

namespace SITL {

/*
  class to describe a motor position
 */
class Motor {
public:
    float angle;
    float yaw_factor;
    uint8_t servo;
    uint8_t display_order;

    // support for tilting motors
    int8_t roll_servo = -1;
    float roll_min, roll_max;
    int8_t pitch_servo = -1;
    float pitch_min, pitch_max;

    // support for servo slew rate
    enum {SERVO_NORMAL, SERVO_RETRACT} servo_type;
    float servo_rate = 0.24; // seconds per 60 degrees
    uint64_t last_change_usec;
    float last_roll_value, last_pitch_value;

    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order) :
        servo(_servo), // what servo output drives this motor
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        display_order(_display_order) // order for clockwise display
    {}

    /*
      alternative constructor for tiltable motors
     */
    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order,
          int8_t _roll_servo, float _roll_min, float _roll_max,
          int8_t _pitch_servo, float _pitch_min, float _pitch_max) :
        servo(_servo), // what servo output drives this motor
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        display_order(_display_order), // order for clockwise display
        roll_servo(_roll_servo),
        roll_min(_roll_min),
        roll_max(_roll_max),
        pitch_servo(_pitch_servo),
        pitch_min(_pitch_min),
        pitch_max(_pitch_max)
    {}

    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {

    ar & BOOST_SERIALIZATION_NVP(angle);
    ar & BOOST_SERIALIZATION_NVP(yaw_factor);
    ar & BOOST_SERIALIZATION_NVP(servo);
    ar & BOOST_SERIALIZATION_NVP(display_order);
    ar & BOOST_SERIALIZATION_NVP(roll_servo);
    ar & BOOST_SERIALIZATION_NVP(roll_min);
    ar & BOOST_SERIALIZATION_NVP(roll_max);
    ar & BOOST_SERIALIZATION_NVP(pitch_servo);
    ar & BOOST_SERIALIZATION_NVP(pitch_min);
    ar & BOOST_SERIALIZATION_NVP(pitch_max);
    ar & BOOST_SERIALIZATION_NVP(servo_rate);
    ar & BOOST_SERIALIZATION_NVP(last_change_usec);
    ar & BOOST_SERIALIZATION_NVP(last_roll_value);
    ar & BOOST_SERIALIZATION_NVP(last_pitch_value);

    }


    void calculate_forces(const struct sitl_input &input,
                          float thrust_scale,
                          uint8_t motor_offset,
                          Vector3f &rot_accel, // rad/sec
                          Vector3f &body_thrust); // Z is down

    uint16_t update_servo(uint16_t demand, uint64_t time_usec, float &last_value);

    // calculate current and voltage
    void current_and_voltage(const struct sitl_input &input, float &voltage, float &current, uint8_t motor_offset);
};

}
