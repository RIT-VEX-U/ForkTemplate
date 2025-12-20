#pragma once

#include "core.h"
#include "vex.h"

// ================ INPUTS ================
// Digital sensors

extern vex::controller Con;
// Analog sensors

// ================ OUTPUTS ================
// Motors

// Pneumatics

// ================ SUBSYSTEMS ================
extern vex::motor front_left_top;
extern vex::motor front_left_bottom;

extern vex::motor front_right_top;
extern vex::motor front_right_bottom;

extern vex::motor back_left_top;
extern vex::motor back_left_bottom;

extern vex::motor back_right_top;
extern vex::motor back_right_bottom;

// ================ UTILS ================

void robot_init();
