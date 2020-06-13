#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class RCMapper {
public:
    RCMapper();

    /* Do not allow copies */
    RCMapper(const RCMapper &other) = delete;
    RCMapper &operator=(const RCMapper&) = delete;

    // get singleton instance
    static RCMapper *get_singleton()
    {
        return _singleton;
    }

    /// roll - return input channel number for roll / aileron input
    uint8_t roll() const { return _ch_roll; }

    /// pitch - return input channel number for pitch / elevator input
    uint8_t pitch() const { return _ch_pitch; }

    /// throttle - return input channel number for throttle input
    uint8_t throttle() const { return _ch_throttle; }

    /// yaw - return input channel number for yaw / rudder input
    uint8_t yaw() const { return _ch_yaw; }

    /// forward - return input channel number for forward input
    uint8_t forward() const { return _ch_forward; }

    /// lateral - return input channel number for lateral input
    uint8_t lateral() const { return _ch_lateral; }

    static const struct AP_Param::GroupInfo var_info[];

    friend class boost::serialization::access; 
    // When the class Archive corresponds to an output archive, the 
    // & operator is defined similar to <<.  Likewise, when the class Archive 
    // is a type of input archive the & operator is defined similar to >>. 
    template<class Archive> 
    void serialize(Archive & ar, const unsigned int version) 
    { 
        ::printf("serializing -> %s\n", __PRETTY_FUNCTION__);     
        ar & BOOST_SERIALIZATION_NVP(_ch_roll);
        ar & BOOST_SERIALIZATION_NVP(_ch_pitch);
        ar & BOOST_SERIALIZATION_NVP(_ch_yaw);
        ar & BOOST_SERIALIZATION_NVP(_ch_throttle);
        ar & BOOST_SERIALIZATION_NVP(_ch_forward);
        ar & BOOST_SERIALIZATION_NVP(_ch_lateral);
    
    }

private:
    // channel mappings
    AP_Int8 _ch_roll;
    AP_Int8 _ch_pitch;
    AP_Int8 _ch_yaw;
    AP_Int8 _ch_throttle;
    AP_Int8 _ch_forward;
    AP_Int8 _ch_lateral;
    static RCMapper *_singleton;
};

namespace AP
{
RCMapper *rcmap();
};
