#include "robot-config.h"
#include "competition/autonomous.h"
#include "competition/opcontrol.h"

#include "core.h"
#include "core/subsystems/screen/legacy.h"
#include "core/subsystems/screen/legacy_bridge.h"
#include "core/subsystems/screen/screen_controller.h"
#include "core/utils/initializer.h"
#include <v5_api.h>
#include <v5_apitypes.h>

vex::brain Brain;

vex::motor mot(vex::PORT19);
vex::motor mot2(vex::PORT20);

std::vector<Initialization> inits = {
    RED_INIT("Red", []() {
        printf("Red\n");
    }),
    BLUE_INIT("Blue", []() {
        printf("Blue\n");
    }),
    NEUTRAL_INIT("Skills", []() {
        printf("Skills\n");
    })
};

LegacyScreen::LegacyPage init_page, match_page;

Initializer initializer(
    inits,
    LegacyScreen::InitializerPage::timed_selector(20),
    LegacyScreen::pre_initialize(Brain, initializer, &init_page),
    []() {
        vexDelay(5000); // Time delay to mimic initializing things
        ScreenController::set((match_page = LegacyScreen::LegacyPage(Brain.Screen, {
            new LegacyScreen::StatsPage({ {"motor", mot} }),
            new LegacyScreen::StatsPage({ {"motor 2", mot2} })
        })).handle());
    }
);