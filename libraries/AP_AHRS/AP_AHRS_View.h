#pragma once

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
 *  AHRS View class - for creating a 2nd view of the vehicle attitude
 *
 */

#include "AP_AHRS.h"

class AP_AHRS_View
{
public:
    // Constructor
    AP_AHRS_View(AP_AHRS &ahrs, enum Rotation rotation, float pitch_trim_deg=0);

    // update state
    void update(bool skip_ins_update=false);

    // empty virtual destructor
    virtual ~AP_AHRS_View() {}

    // return a smoothed and corrected gyro vector
    const Vector3f &get_gyro(void) const {
        return gyro;
    }

    // return a smoothed and corrected gyro vector using the latest ins data (which may not have been consumed by the EKF yet)
    Vector3f get_gyro_latest(void) const;

    // return a DCM rotation matrix representing our current attitude in this view
    const Matrix3f &get_rotation_body_to_ned(void) const {
        return rot_body_to_ned;
    }

    // return a Quaternion representing our current attitude in this view
    void get_quat_body_to_ned(Quaternion &quat) const {
        quat.from_rotation_matrix(rot_body_to_ned);
    }

    // apply pitch trim
    void set_pitch_trim(float trim_deg);

    // helper trig value accessors
    float cos_roll() const {
        return trig.cos_roll;
    }
    float cos_pitch() const {
        return trig.cos_pitch;
    }
    float cos_yaw() const {
        return trig.cos_yaw;
    }
    float sin_roll() const {
        return trig.sin_roll;
    }
    float sin_pitch() const {
        return trig.sin_pitch;
    }
    float sin_yaw() const {
        return trig.sin_yaw;
    }


    /*
      wrappers around ahrs functions which pass-thru directly. See
      AP_AHRS.h for description of each function
     */
    bool get_position(struct Location &loc) const WARN_IF_UNUSED {
        return ahrs.get_position(loc);
    }

    Vector3f wind_estimate(void) {
        return ahrs.wind_estimate();
    }

    bool airspeed_estimate(float &airspeed_ret) const WARN_IF_UNUSED {
        return ahrs.airspeed_estimate(airspeed_ret);
    }

    bool airspeed_estimate_true(float &airspeed_ret) const WARN_IF_UNUSED {
        return ahrs.airspeed_estimate_true(airspeed_ret);
    }

    float get_EAS2TAS(void) const {
        return ahrs.get_EAS2TAS();
    }

    Vector2f groundspeed_vector(void) {
        return ahrs.groundspeed_vector();
    }

    bool get_velocity_NED(Vector3f &vec) const WARN_IF_UNUSED {
        return ahrs.get_velocity_NED(vec);
    }

    bool get_expected_mag_field_NED(Vector3f &ret) const WARN_IF_UNUSED {
        return ahrs.get_expected_mag_field_NED(ret);
    }

    bool get_relative_position_NED_home(Vector3f &vec) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_NED_home(vec);
    }

    bool get_relative_position_NED_origin(Vector3f &vec) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_NED_origin(vec);
    }

    bool get_relative_position_NE_home(Vector2f &vecNE) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_NE_home(vecNE);
    }

    bool get_relative_position_NE_origin(Vector2f &vecNE) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_NE_origin(vecNE);
    }

    void get_relative_position_D_home(float &posD) const {
        ahrs.get_relative_position_D_home(posD);
    }

    bool get_relative_position_D_origin(float &posD) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_D_origin(posD);
    }

    float groundspeed(void) {
        return ahrs.groundspeed();
    }

    const Vector3f &get_accel_ef_blended(void) const {
        return ahrs.get_accel_ef_blended();
    }

    uint32_t getLastPosNorthEastReset(Vector2f &pos) WARN_IF_UNUSED {
        return ahrs.getLastPosNorthEastReset(pos);
    }

    uint32_t getLastPosDownReset(float &posDelta) WARN_IF_UNUSED {
        return ahrs.getLastPosDownReset(posDelta);
    }

    // rotate a 2D vector from earth frame to body frame
    // in result, x is forward, y is right
    Vector2f rotate_earth_to_body2D(const Vector2f &ef_vector) const;

    // rotate a 2D vector from earth frame to body frame
    // in input, x is forward, y is right
    Vector2f rotate_body_to_earth2D(const Vector2f &bf) const;

    // return the average size of the roll/pitch error estimate
    // since last call
    float get_error_rp(void) const {
        return ahrs.get_error_rp();
    }

    // return the average size of the yaw error estimate
    // since last call
    float get_error_yaw(void) const {
        return ahrs.get_error_yaw();
    }

    friend class boost::serialization::access; 
    // When the class Archive corresponds to an output archive, the 
    // & operator is defined similar to <<.  Likewise, when the class Archive 
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive> 
    void serialize(Archive & ar, const unsigned int version) 
    { 
        ::printf("serializing -> %s\n", __PRETTY_FUNCTION__);     
        
        ar & BOOST_SERIALIZATION_NVP(roll);
        ar & BOOST_SERIALIZATION_NVP(pitch);
        ar & BOOST_SERIALIZATION_NVP(yaw);
        ar & BOOST_SERIALIZATION_NVP(roll_sensor);
        ar & BOOST_SERIALIZATION_NVP(pitch_sensor);
        ar & BOOST_SERIALIZATION_NVP(yaw_sensor);

        //ar.template register_type<AP_AHRS>(); 
        //ar & BOOST_SERIALIZATION_NVP(ahrs); // serializer things this is a 'pointer conflict' between AP_AHRS and AP_AHRS_View https://www.boost.org/doc/libs/1_37_0/libs/serialization/doc/exceptions.html#pointer_conflict
        ar & BOOST_SERIALIZATION_NVP(rot_view);
        ar & BOOST_SERIALIZATION_NVP(rot_view_T);
        ar & BOOST_SERIALIZATION_NVP(rot_body_to_ned);
        ar & BOOST_SERIALIZATION_NVP(gyro);
        ar & BOOST_SERIALIZATION_NVP(trig);
        ar & BOOST_SERIALIZATION_NVP(y_angle);
        ar & BOOST_SERIALIZATION_NVP(_pitch_trim_deg);

    }

    float roll;
    float pitch;
    float yaw;
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

private:
    const enum Rotation rotation;
    AP_AHRS &ahrs;

    // body frame rotation for this View
    Matrix3f rot_view;
    // transpose of rot_view
    Matrix3f rot_view_T;
    Matrix3f rot_body_to_ned;
    Vector3f gyro;

    struct {
        float cos_roll;
        float cos_pitch;
        float cos_yaw;
        float sin_roll;
        float sin_pitch;
        float sin_yaw;

        template<class Archive> 
        void serialize(Archive & ar, const unsigned int version) 
        { 
            ::printf("serializing -> %s\n", __PRETTY_FUNCTION__);                 
            ar & BOOST_SERIALIZATION_NVP(cos_roll);
            ar & BOOST_SERIALIZATION_NVP(cos_pitch);
            ar & BOOST_SERIALIZATION_NVP(cos_yaw);
            ar & BOOST_SERIALIZATION_NVP(sin_roll);
            ar & BOOST_SERIALIZATION_NVP(sin_pitch);
            ar & BOOST_SERIALIZATION_NVP(sin_yaw);
        }

    } trig;

    float y_angle;
    float _pitch_trim_deg;
};
