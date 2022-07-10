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
#include "Plane.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Plane class
 */
Plane::Plane(void)
    : logger(g.log_bitmask)
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
}

Plane::~Plane(void){}

#include <AP_HAL_SITL/AP_HAL_SITL.h>

extern const HAL_SITL& hal_sitl;


HAL_SITL* Plane::singleton(void){
  //return (HAL_SITL&)hal;
  return &(HAL_SITL&)hal_sitl; // buzz is this really right ?
}


Plane plane;
AP_Vehicle& vehicle = plane;


#include <emscripten/bind.h>

using namespace emscripten;


//JS Binding code
// if we called 'new ... Plane from JS, we get => AP_HAL::panic("Too many schedulers");
EMSCRIPTEN_BINDINGS(Plane_example2) {
  class_<Plane>("Plane")
    .constructor()
    .function("any_failsafe_triggered", &Plane::any_failsafe_triggered)
    .function("terrain_disabled", &Plane::terrain_disabled)

    //.function("singleton", &Plane::singleton, allow_raw_pointers())
    //.property("x", &Plane::getX, &Plane::setX)
    //.class_function("getStringFromInstance", &Plane::getStringFromInstance)
    //.function("singleton", &singleton, allow_raw_pointers())

    ;
}
