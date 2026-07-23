#include "vex.h"

#include "robot-config.h"

#include "competition/autonomous.h"
#include "competition/opcontrol.h"

#include "core/units/units.h"
vex::competition comp;

/**
 * Entry point to the program. No code should be placed here;
 * instead use competition/opcontrol.cpp and
 * competition/autonomous.cpp
 */
int main() {
  constexpr Length length = 1_in;
  constexpr Time time = 1_s;
  constexpr Velocity vel = length / time;
  static_assert(vel == 1_inps, "shit hit the fan");

  comp.autonomous(autonomous);
  comp.drivercontrol(opcontrol);

  robot_init();
}
