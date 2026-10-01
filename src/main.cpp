#include "config/mod.hpp"
#include "crust/mod.hpp"
#include "competition/autonomous.h"
#include "competition/opcontrol.h"

void autonomous() {
    // Autonomous routine
}

void skills() {
    // Skills routine
}

void opcontrol() {
    // Driver control loop
}

void (*autonomous_ptr)() = autonomous;
void (*opcontrol_ptr)() = opcontrol;

int main() {
    // 1. Initialize crust runtime (clock, delay, and RTOS task hooks)
    crust::init();

    // 2. Run robot initializer (pre-autonomous GUI / routine selection)
    if (config::robot.initializer) {
        config::robot.initializer->initialize();
    }

    // 3. Register competition callbacks
    config::robot.competition.autonomous(autonomous_ptr);
    config::robot.competition.drivercontrol(opcontrol_ptr);

    // 4. Background competition loop
    while (true) {
        core::delay_ms(100);
    }
}