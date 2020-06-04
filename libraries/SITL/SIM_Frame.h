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
  multicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"

#include <SITL/Serialize.h>

namespace SITL {

/*
  class to describe a multicopter frame type
 */
class Frame {
public:
    const char *name;
    uint8_t num_motors;
    Motor *motors;

    Frame(const char *_name,
          uint8_t _num_motors,
          Motor *_motors) :
        name(_name),
        num_motors(_num_motors),
        motors(_motors) {}

    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {

      // ar & BOOST_SERIALIZATION_NVP(name);
        ar & BOOST_SERIALIZATION_NVP( num_motors);
        ar & BOOST_SERIALIZATION_NVP(motors);
        ar & BOOST_SERIALIZATION_NVP( terminal_velocity);
        ar & BOOST_SERIALIZATION_NVP( terminal_rotation_rate);
        ar & BOOST_SERIALIZATION_NVP( thrust_scale);
        ar & BOOST_SERIALIZATION_NVP( motor_offset);

    }

    // find a frame by name
    static Frame *find_frame(const char *name);
    
    // initialise frame
    void init(float mass, float hover_throttle, float terminal_velocity, float terminal_rotation_rate);

    // calculate rotational and linear accelerations
    void calculate_forces(const Aircraft &aircraft,
                          const struct sitl_input &input,
                          Vector3f &rot_accel, Vector3f &body_accel, float* rpm);
    
    float terminal_velocity;
    float terminal_rotation_rate;
    float thrust_scale;
    uint8_t motor_offset;

    // calculate current and voltage
    void current_and_voltage(const struct sitl_input &input, float &voltage, float &current);
};
}
