#include "vex.h"
#include <cmath>
#include <vex_brain.h>
#include <vex_task.h>

vex::competition comp;

/**
 * Entry point to the program. No code should be placed here;
 * instead use competition/opcontrol.cpp and
 * competition/autonomous.cpp
*/
int main() {
    vex::brain brain = vex::brain();
    brain.Screen.print("aweoifj\n");

    double x = 2.0;
    double cosx = cos(x);

    brain.Screen.print("helloa %f\n", cosx);
}
