#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "SITL_State.h"

#include <fstream>

// without som sort of boost reference fist, the next ones errror
//#include <boost/regex.hpp>

// include headers that implement a archive in simple text format
//#include <boost/archive/text_oarchive.hpp>
//#include <boost/archive/text_iarchive.hpp>

#include <boost/exception/exception.hpp>
#include <boost/current_function.hpp>
#if !defined( BOOST_THROW_EXCEPTION )
#define BOOST_THROW_EXCEPTION(x) ::boost::exception_detail::throw_exception_(x,BOOST_CURRENT_FUNCTION,__FILE__,__LINE__)
#endif

class HAL_SITL : public AP_HAL::HAL {
public:
    HAL_SITL();
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;
    static void actually_reboot();

private:
    //friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    //template<class Archive>
    //void serialize(Archive & ar, const unsigned int version)
    //{
        //ar & _sitl_state;
        //ar & minutes;
        //ar & seconds;
    //}

    HALSITL::SITL_State *_sitl_state;

    void setup_signal_handlers() const;


    static void exit_signal_handler(int);
};

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
