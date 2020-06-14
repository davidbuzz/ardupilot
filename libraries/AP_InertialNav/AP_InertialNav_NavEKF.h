/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */
#pragma once

#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

class AP_InertialNav_NavEKF : public AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav_NavEKF(AP_AHRS_NavEKF &ahrs) :
        AP_InertialNav(),
        _ahrs_ekf(ahrs)
        {}

    /**
       update internal state
    */
    void        update(bool high_vibes = false) override;

    /**
     * get_filter_status - returns filter status as a series of flags
     */
    nav_filter_status get_filter_status() const override;

    /**
     * get_position - returns the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_position(int32_t, int32_t)
     *
     * @return
     */
    const Vector3f&    get_position() const override;

    /**
     * get_velocity - returns the current velocity in cm/s
     *
     * @return velocity vector:
     *      		.x : latitude  velocity in cm/s
     * 				.y : longitude velocity in cm/s
     * 				.z : vertical  velocity in cm/s
     */
    const Vector3f&    get_velocity() const override;

    /**
     * get_speed_xy - returns the current horizontal speed in cm/s
     *
     * @returns the current horizontal speed in cm/s
     */
    float        get_speed_xy() const override;

    /**
     * get_altitude - get latest altitude estimate in cm
     * @return
     */
    float       get_altitude() const override;

    /**
     * get_velocity_z - returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const override;

    friend class boost::serialization::access; 
    // When the class Archive corresponds to an output archive, the 
    // & operator is defined similar to <<.  Likewise, when the class Archive 
    // is a type of input archive the & operator is defined similar to >>. 
    template<class Archive> 
    void serialize(Archive & ar, const unsigned int version) 
    { 
        //ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(AP_InertialNav);// AP_InertialNav is virtual and has no private vars to speak of
        ::printf("serializing -> %s\n", __PRETTY_FUNCTION__);     

        ar & BOOST_SERIALIZATION_NVP(_relpos_cm);
        ar & BOOST_SERIALIZATION_NVP(_velocity_cm);
        ar & BOOST_SERIALIZATION_NVP(_ahrs_ekf); //error: ‘class AP_AHRS_NavEKF’ has no member named ‘serialize’
    }

private:
    Vector3f _relpos_cm;   // NEU
    Vector3f _velocity_cm; // NEU
    AP_AHRS_NavEKF &_ahrs_ekf;
};
