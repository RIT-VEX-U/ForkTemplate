#pragma once

#include "config/gains.h"
#include "config/ports.h"
#include "config/specs.h"
#include "crust/brain.h"
#include "crust/competition.h"
#include "crust/controller.h"
#include "crust/motor.h"
#include "crust/sensor.h"
#include "mantle/display/legacy.h"
#include "mantle/display/screen_controller.h"
#include "mantle/initializer.h"
#include "mantle/motor.h"
#include "mantle/subsystems/flywheel.h"
#include "mantle/subsystems/lift.h"
#include "mantle/subsystems/tank_drive.h"
#include <memory>

namespace config {

/**
 * Central hardware and subsystem bundle wiring the physical robot.
 */
class RobotHardware {
public:
    // Core physical controllers & brain
    vex::brain brain;
    crust::Competition competition;
    crust::V5Controller controller;
    crust::BrainScreen screen;

    // Drivetrain motors
    std::shared_ptr<crust::V5Motor> left1, left2, left3, left4;
    std::shared_ptr<crust::V5Motor> right1, right2, right3, right4;
    mantle::MotorGroup left_drive;
    mantle::MotorGroup right_drive;
    vex::motor_group left_motors;
    vex::motor_group right_motors;

    // Lift motors & subsystem
    std::shared_ptr<crust::V5Motor> lift_left, lift_right;
    mantle::MotorGroup lift_motors;

    // Robot physical specification
    RobotSpecs specs;

    // High-level subsystems
    std::shared_ptr<mantle::TankDrive> drive;
    std::shared_ptr<mantle::Initializer> initializer;

    RobotHardware();
};

extern RobotHardware robot;

} // namespace config
