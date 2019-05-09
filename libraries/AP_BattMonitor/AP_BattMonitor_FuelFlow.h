#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

class AP_BattMonitor_FuelFlow : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_FuelFlow(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override {}

    bool zero_consumed();

private:
    static void irq_handler(void);

    struct IrqState {
        uint32_t pulse_count;
        uint32_t total_us;
        uint32_t last_pulse_us;
    };
    static struct IrqState irq_state;

    int8_t last_pin = -1;
};
