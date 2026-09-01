#include "vex.h"

#include "robot-config.h"

#include "competition/autonomous.h"
#include "competition/opcontrol.h"

vex::competition comp;

/**
 * Entry point to the program. No code should be placed here;
 * instead use competition/opcontrol.cpp and
 * competition/autonomous.cpp
 */
int main() {
  initializer.initialize();

  comp.autonomous(autonomous_ptr);
  comp.drivercontrol(opcontrol_ptr);
}
