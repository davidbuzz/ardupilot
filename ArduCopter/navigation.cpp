#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);

    flightmode->update_navigation();

    // allow the nav subsystem to re-determine 'nearest' waypoint
    do_update_wp_counts(); 
}

void Copter::do_update_wp_counts(){

    // get data from mission object and put it into ahrs object, as ahrs has a singleton that LUA can use.
    mode_auto.mission.find_nearest_waypoint(mode_auto.mission._nearest);
    //ahrs._nearest = mode_auto.mission._nearest;

}

// distance between vehicle and home in cm
uint32_t Copter::home_distance()
{
    if (position_ok()) {
        _home_distance = current_loc.get_distance(ahrs.get_home()) * 100;
    }
    return _home_distance;
}

// The location of home in relation to the vehicle in centi-degrees
int32_t Copter::home_bearing()
{
    if (position_ok()) {
        _home_bearing = current_loc.get_bearing_to(ahrs.get_home());
    }
    return _home_bearing;
}
