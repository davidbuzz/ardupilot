#pragma once

//#include "AP_Param.h"

#include "vartypes.h"

#include <SITL/Serialize.h>


//class AP_Param;

/// Template class for scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
///
/// @tparam T			The scalar type of the variable
/// @tparam PT			The AP_PARAM_* type
///
template<typename T, ap_var_type PT>
class AP_ParamT : public AP_Param
{
public:
    static const ap_var_type        vtype = PT;
    //bool is_empty_constructor = false;

    // only boost uses this during de-serialize, and we want it to default to zero, not random ram
    AP_ParamT() { 
        ::printf("AP_Param-T used_empty_constructor TRUE\n");
        _value = 0;
        used_empty_constructor = true;
        }

    /// Value getter
    ///
    const T &get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void set(const T &v) {
        _value = v;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
                // method 1 : invoke base class serialization
        //boost::serialization::base_object<AP_Param>(*this);
        // method 2 : explicitly register base/derived relationship
        //boost::serialization::void_cast_register<base, derived>();
        
      ar & BOOST_SERIALIZATION_NVP(_value);
    }

    // set a parameter that is an ENABLE param
    void set_enable(const T &v) {
        if (v != _value) {
            invalidate_count();
        }
        _value = v;
    }
    
    /// Sets if the parameter is unconfigured
    ///
    void set_default(const T &v) {
        if (!configured()) {
            set(v);
        }
    }

    /// Value setter - set value, tell GCS
    ///
    void set_and_notify(const T &v) {
// We do want to compare each value, even floats, since it being the same here
// is the result of previously setting it.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
        if (v != _value) {
#pragma GCC diagnostic pop
            set(v);
            notify();
        }
    }

    /// Combined set and save
    ///
    void set_and_save(const T &v) {
        bool force = fabsf((float)(_value - v)) < FLT_EPSILON;
        set(v);
        save(force);
    }

    /// Combined set and save, but only does the save if the value if
    /// different from the current ram value, thus saving us a
    /// scan(). This should only be used where we have not set() the
    /// value separately, as otherwise the value in EEPROM won't be
    /// updated correctly.
    void set_and_save_ifchanged(const T &v) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
        if (v == _value) {
#pragma GCC diagnostic pop
            return;
        }
        set(v);
        save(true);
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator const T &() const {
        return _value;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AP_ParamT<T,PT>& operator= (const T &v) {
        if ( used_empty_constructor == true) { 
            _value = 0; 
            ::printf("AP_Param-T used_empty_constructor ZEROd and set FALSE\n");
            used_empty_constructor=false;
        } else { 
            _value = v;
        }
        return *this;
    }

    /// bit ops on parameters
    ///
    AP_ParamT<T,PT>& operator |=(const T &v) {
        _value |= v;
        return *this;
    }

    AP_ParamT<T,PT>& operator &=(const T &v) {
        _value &= v;
        return *this;
    }

    AP_ParamT<T,PT>& operator +=(const T &v) {
        _value += v;
        return *this;
    }

    AP_ParamT<T,PT>& operator -=(const T &v) {
        _value -= v;
        return *this;
    }

    /// AP_ParamT types can implement AP_Param::cast_to_float
    ///
    float cast_to_float(void) const {
        return (float)_value;
    }

protected:
    T _value;
};
//BOOST_CLASS_EXPORT(AP_ParamT)


/// Template class for non-scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
///
/// @tparam T			The scalar type of the variable
/// @tparam PT			AP_PARAM_* type
///
template<typename T, ap_var_type PT>
class AP_ParamV : public AP_Param
{
public:

    static const ap_var_type        vtype = PT;

    /// Value getter
    ///
    const T &get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void set(const T &v) {
        _value = v;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(_value);
    }

    /// Value setter - set value, tell GCS
    ///
    void set_and_notify(const T &v) {
        if (v != _value) {
            set(v);
            notify();
        }
    }

    /// Combined set and save
    ///
    void set_and_save(const T &v) {
        bool force = (_value != v);
        set(v);
        save(force);
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator const T &() const {
        return _value;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AP_ParamV<T,PT>& operator=(const T &v) {

        if ( used_empty_constructor == true) { 
            //_value = (Vector3<float>)0.0; 
            ::printf("AP_Param-VVV used_empty_constructor ZEROd and set FALSE\n");
            used_empty_constructor=false;
        } else { 
            _value = v;
        }
        return *this;
    }

protected:
    T        _value;
};
//BOOST_CLASS_EXPORT(AP_ParamV)


/// Template class for array variables.
///
/// Objects created using this template behave like arrays of the type T,
/// but are stored like single variables.
///
/// @tparam T           The scalar type of the variable
/// @tparam N           number of elements
/// @tparam PT          the AP_PARAM_* type
///
template<typename T, uint8_t N, ap_var_type PT>
class AP_ParamA : public AP_Param
{
public:

    static const ap_var_type vtype = PT;

    /// Array operator accesses members.
    ///
    /// @note It would be nice to range-check i here, but then what would we return?
    ///
    const T & operator[](uint8_t i) {
        return _value[i];
    }

    const T & operator[](int8_t i) {
        return _value[(uint8_t)i];
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(_value);
    }

    /// Value getter
    ///
    /// @note   Returns zero for index values out of range.
    ///
    T get(uint8_t i) const {
        if (used_empty_constructor) { 
            ::printf("AP_Param-AAA just returned 0\n");
            return 0;
            }
        if (i < N) {
            return _value[i];
        } else {
            return (T)0;
        }
    }

    /// Value setter
    ///
    /// @note   Attempts to set an index out of range are discarded.
    ///
    void  set(uint8_t i, const T &v) {
        if (i < N) {
            _value[i] = v;
        }
    }

protected:
    T _value[N];
};
//BOOST_CLASS_EXPORT(AP_ParamA)


#include "vartemplates.h"
