#pragma once

#include <cstdint>

namespace config {

// V5 Smart Ports (1-21)
constexpr int32_t PORT_LEFT_DRIVE_1 = 1;
constexpr int32_t PORT_LEFT_DRIVE_2 = 2;
constexpr int32_t PORT_LEFT_DRIVE_3 = 3;
constexpr int32_t PORT_LEFT_DRIVE_4 = 4;

constexpr int32_t PORT_RIGHT_DRIVE_1 = 5;
constexpr int32_t PORT_RIGHT_DRIVE_2 = 6;
constexpr int32_t PORT_RIGHT_DRIVE_3 = 7;
constexpr int32_t PORT_RIGHT_DRIVE_4 = 8;

constexpr int32_t PORT_LIFT_LEFT = 9;
constexpr int32_t PORT_LIFT_RIGHT = 10;

constexpr int32_t PORT_FLYWHEEL = 11;
constexpr int32_t PORT_INERTIAL = 12;

} // namespace config
