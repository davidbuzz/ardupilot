#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "RCInput.h"

#include <sys/types.h>

#ifndef _WIN32
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#endif

#include <vector>

#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <SITL/SITL.h>


//...

class MingW {
public:
    MingW(bool initter);
    ~MingW();

    bool thingy(uint16_t backlog) const;

    // accept a new connection. Only valid for TCP connections after
    // listen has been used. A new socket is returned
    //SocketAPM *accept(uint32_t timeout_ms);

private:
    bool yesno;

    const MingW& get_HAL();
};



#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
