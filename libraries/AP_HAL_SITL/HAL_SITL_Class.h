#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "SITL_State.h"

#include <SITL/Serialize.h>
#include <stdio.h>

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
        //::printf("serializing -> %s\n", __PRETTY_FUNCTION__);
        //ar & _sitl_state;
        //ar & minutes;
        //ar & seconds;
    //}

    HALSITL::SITL_State *_sitl_state;

    void setup_signal_handlers() const;


    static void exit_signal_handler(int);
};

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
