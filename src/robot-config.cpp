#include "robot-config.h"
#include "competition/autonomous.h"
#include "competition/opcontrol.h"


vex::brain brain;
vex::competition competition;
vex::controller controller;


std::vector<Initialization> inits = {};

LegacyScreen::LegacyPage init_page, match_page;
Initializer initializer(inits, LegacyScreen::InitializerPage::timed_selector(20),
LegacyScreen::pre_initialize(brain, initializer, &init_page), []() {
    // Initialization code here
});