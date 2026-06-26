#include "competition/autonomous.h"
#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

vex::competition comp;

/**
 * Entry point to the program. No code should be placed here;
 * instead use competition/opcontrol.cpp and
 * competition/autonomous.cpp
 */
int main() {
    comp.autonomous(autonomous);
    comp.drivercontrol(opcontrol);

    robot_init();
}
