#pragma once

#include "vex.h"

namespace crust {

/**
 * Competition state manager wrapping vex::competition.
 */
class Competition {
private:
    mutable vex::competition comp;

public:
    Competition() = default;

    void autonomous(void (*callback)()) {
        comp.autonomous(callback);
    }

    void drivercontrol(void (*callback)()) {
        comp.drivercontrol(callback);
    }

    bool is_autonomous() const {
        return comp.isAutonomous();
    }

    bool is_driver_control() const {
        return comp.isDriverControl();
    }

    bool is_enabled() const {
        return comp.isEnabled();
    }

    vex::competition& raw_competition() {
        return comp;
    }
};

} // namespace crust
