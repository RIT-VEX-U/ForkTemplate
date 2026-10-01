#pragma once

#include "core/controls/feedback_base.h"
#include "core/controls/pid.h"

namespace mantle {

/**
 * Robot specifications and kinematic dimensions.
 * Distances are in inches.
 */
struct RobotSpecs {
    double robot_radius = 9.0;
    double odom_wheel_diam = 2.75;
    double odom_gear_ratio = 1.0;
    double dist_between_wheels = 12.0;
    double drive_correction_cutoff = 2.0;
    Feedback* drive_feedback = nullptr;
    Feedback* turn_feedback = nullptr;
    PID::pid_config_t correction_pid = {
        .p = 0.05,
        .i = 0.0,
        .d = 0.005,
        .error_method = PID::ANGULAR
    };
};

using robot_specs_t = RobotSpecs;

} // namespace mantle

using robot_specs_t = mantle::RobotSpecs;
