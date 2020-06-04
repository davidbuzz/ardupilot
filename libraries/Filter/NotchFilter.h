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
#pragma once

/*
  notch filter with settable sample rate, center frequency, bandwidth and attenuation

  Design by Leonard Hall
 */

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>
#include <AP_Param/AP_Param.h>


// without som sort of boost reference fist, the next ones errror
#include <boost/regex.hpp>
#include <boost/exception/exception.hpp>
#include <boost/current_function.hpp>
#if !defined( BOOST_THROW_EXCEPTION )
#define BOOST_THROW_EXCEPTION(x) ::boost::exception_detail::throw_exception_(x,BOOST_CURRENT_FUNCTION,__FILE__,__LINE__)
#endif
// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


template <class T>
class NotchFilter {
public:
    // set parameters
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
    void init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q);
    T apply(const T &sample);
    void reset();

    // calculate attenuation and quality from provided center frequency and bandwidth
    static void calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float& A, float& Q); 


    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
    ar & BOOST_SERIALIZATION_NVP(initialised);

    ar & BOOST_SERIALIZATION_NVP(b0);
    ar & BOOST_SERIALIZATION_NVP(b1);
    ar & BOOST_SERIALIZATION_NVP(b2);
    ar & BOOST_SERIALIZATION_NVP(a1);
    ar & BOOST_SERIALIZATION_NVP(a2);
    ar & BOOST_SERIALIZATION_NVP(a0_inv);

    ar & BOOST_SERIALIZATION_NVP(ntchsig);
    ar & BOOST_SERIALIZATION_NVP(ntchsig1);
    ar & BOOST_SERIALIZATION_NVP(ntchsig2);
    ar & BOOST_SERIALIZATION_NVP(signal2);
    ar & BOOST_SERIALIZATION_NVP(signal1);

    } 

private:

    bool initialised;
    float b0, b1, b2, a1, a2, a0_inv;
    T ntchsig, ntchsig1, ntchsig2, signal2, signal1;
};

/*
  notch filter enable and filter parameters
 */
class NotchFilterParams {
public:
    NotchFilterParams(void);
    static const struct AP_Param::GroupInfo var_info[];

    float center_freq_hz(void) const { return _center_freq_hz; }
    float bandwidth_hz(void) const { return _bandwidth_hz; }
    float attenuation_dB(void) const { return _attenuation_dB; }
    uint8_t enabled(void) const { return _enable; }

    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
    ar & BOOST_SERIALIZATION_NVP(_enable);
    ar & BOOST_SERIALIZATION_NVP(_center_freq_hz);
    ar & BOOST_SERIALIZATION_NVP(_bandwidth_hz);
    ar & BOOST_SERIALIZATION_NVP(_attenuation_dB);
    }    

protected:
    AP_Int8 _enable;
    AP_Float _center_freq_hz;
    AP_Float _bandwidth_hz;
    AP_Float _attenuation_dB;
};

typedef NotchFilter<float> NotchFilterFloat;
typedef NotchFilter<Vector3f> NotchFilterVector3f;

