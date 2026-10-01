#pragma once

#include "core/controls/feedforward.h"
#include "core/controls/pid.h"

namespace config {

inline PID::pid_config_t drive_pid_gains() {
    return {
        .p = 0.05,
        .i = 0.0,
        .d = 0.005,
        .deadband = 1.0,
        .on_target_time = 0.1,
        .error_method = PID::LINEAR
    };
}

inline PID::pid_config_t turn_pid_gains() {
    return {
        .p = 0.04,
        .i = 0.0,
        .d = 0.004,
        .deadband = 1.5,
        .on_target_time = 0.1,
        .error_method = PID::ANGULAR
    };
}

inline PID::pid_config_t lift_pid_gains() {
    return {
        .p = 0.08,
        .i = 0.001,
        .d = 0.005,
        .deadband = 5.0,
        .on_target_time = 0.05,
        .error_method = PID::LINEAR
    };
}

} // namespace config
