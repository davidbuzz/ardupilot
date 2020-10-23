#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include "Scheduler.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

#ifndef _WIN32
//#include <sys/select.h>
#endif

#include <AP_Param/AP_Param.h>
//#include <SITL/SIM_JSBSim.h>
#include <AP_HAL/utility/Socket.h>

#include <AP_HAL_SITL/mingw-helpers.h> 

extern const AP_HAL::HAL& hal;

//static MingW gg(1);

MingW::MingW(bool initter)
{
}

MingW::~MingW()
{
}

bool MingW::thingy(uint16_t backlog) const
{
hal.console->printf("thingy thingy thingy\n");
return true;
}

const MingW& MingW::get_HAL() {
    static const MingW qq(1); 
   // mm = new MingW(1);
    return qq;
   // return 0;
}

const MingW& mm = MingW(1); // prime it

//static const MingW mm(1); 



#endif
