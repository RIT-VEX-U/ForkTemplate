#include "config/robot.h"
#include "mantle/display/legacy_bridge.h"

namespace config {

RobotHardware robot;

static std::vector<mantle::Initialization> inits = {};
static LegacyScreen::LegacyPage init_page;

RobotHardware::RobotHardware()
    : screen(brain.Screen) {
    // Construct drivetrain motors
    left1 = std::make_shared<crust::V5Motor>(PORT_LEFT_DRIVE_1, false);
    left2 = std::make_shared<crust::V5Motor>(PORT_LEFT_DRIVE_2, false);
    left3 = std::make_shared<crust::V5Motor>(PORT_LEFT_DRIVE_3, false);
    left4 = std::make_shared<crust::V5Motor>(PORT_LEFT_DRIVE_4, false);
    left_drive = mantle::MotorGroup({left1, left2, left3, left4});

    right1 = std::make_shared<crust::V5Motor>(PORT_RIGHT_DRIVE_1, true);
    right2 = std::make_shared<crust::V5Motor>(PORT_RIGHT_DRIVE_2, true);
    right3 = std::make_shared<crust::V5Motor>(PORT_RIGHT_DRIVE_3, true);
    right4 = std::make_shared<crust::V5Motor>(PORT_RIGHT_DRIVE_4, true);
    right_drive = mantle::MotorGroup({right1, right2, right3, right4});

    left_motors(left1->raw_motor(), left2->raw_motor(), left3->raw_motor(), left4->raw_motor());
    right_motors(right1->raw_motor(), right2->raw_motor(), right3->raw_motor(), right4->raw_motor());

    // Construct lift motors
    lift_left = std::make_shared<crust::V5Motor>(PORT_LIFT_LEFT, false);
    lift_right = std::make_shared<crust::V5Motor>(PORT_LIFT_RIGHT, true);
    lift_motors = mantle::MotorGroup({lift_left, lift_right});

    // High level subsystems
    drive = std::make_shared<mantle::TankDrive>(left_drive, right_drive, specs);

    initializer = std::make_shared<mantle::Initializer>(
        inits,
        LegacyScreen::InitializerPage::timed_selector(20),
        [this]() {
            if (initializer) {
                LegacyScreen::pre_initialize(brain, *initializer, &init_page)();
            }
        },
        []() {}
    );
}

} // namespace config
