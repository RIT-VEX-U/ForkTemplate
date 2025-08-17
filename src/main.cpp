#include "vex.h"
#include "robot-config.h"
#include "competition/autonomous.h"
#include "competition/opcontrol.h"
#include <cmath>
#include <vex_task.h>
#include "namespacer.hpp"

vex::competition comp;

/**
 * Entry point to the program. No code should be placed here;
 * instead use competition/opcontrol.cpp and
 * competition/autonomous.cpp
*/
int main() {
    comp.autonomous(autonomous);
    comp.drivercontrol(opcontrol);
    comp.bStopAllTasksBetweenModes = true;

    VDP::Packet packet;

    robot_init();
}
