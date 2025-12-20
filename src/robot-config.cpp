#include "robot-config.h"
#include <vex_global.h>
#include <vex_motor.h>
#include <vex_motorgroup.h>

vex::controller Con;

vex::motor front_left_top(vex::PORT12, vex::gearSetting::ratio6_1, false);
vex::motor front_left_bottom(vex::PORT11, vex::gearSetting::ratio6_1, false);

vex::motor front_right_top(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor front_right_bottom(vex::PORT2, vex::gearSetting::ratio6_1, false);

vex::motor back_left_top(vex::PORT14, vex::gearSetting::ratio6_1, false);
vex::motor back_left_bottom(vex::PORT15, vex::gearSetting::ratio6_1, false);

vex::motor back_right_top(vex::PORT8, vex::gearSetting::ratio6_1, false);
vex::motor back_right_bottom(vex::PORT10, vex::gearSetting::ratio6_1, false);

void robot_init() {
}
