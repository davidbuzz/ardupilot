#pragma once

#include <AP_Arming/AP_Arming.h>
#include <AP_Common/Location.h>

/*
  a plane specific arming class
 */
class AP_Arming_Plane : public AP_Arming
{
public:
    AP_Arming_Plane()
        : AP_Arming()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_Arming_Plane(const AP_Arming_Plane &other) = delete;
    AP_Arming_Plane &operator=(const AP_Arming_Plane&) = delete;

    bool pre_arm_checks(bool report) override;
    bool arm_checks(AP_Arming::Method method) override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    bool disarm() override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

    void update_soft_armed();

/*
    uint32_t get_nearest_wp(Location &loc); // returns wp number of nearest waypoint

    // get the nearest location. 
    const struct Location &get_nearest(void) const {
        return _nearest;
    }
    struct Location _nearest;
*/

protected:
    bool ins_checks(bool report) override;

private:

    void change_arm_state(void);
};

namespace AP {
    AP_Arming &arming();
};
