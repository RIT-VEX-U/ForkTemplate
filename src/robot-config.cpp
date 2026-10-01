#include "robot-config.h"

#include <vex_global.h>
#include <vex_units.h>

#include <cstdio>

#include "competition/autonomous.h"
#include "competition/opcontrol.h"

vex::brain brain;
vex::competition competition;
vex::controller controller;

vex::motor mot1(vex::PORT2);

double mot1_speed = mot1.velocity(vex::velocityUnits::rpm);
double mot1_pos = mot1.position(vex::rotationUnits::deg);
double mot1_volts = 0;

VDP::Field mot1_speed_f("mot1 speed", mot1_speed);
VDP::Field mot1_pos_f("mot1 position", mot1_pos);
VDP::Field mot1_volts_f("mot1 voltage", mot1_volts);

VDP::Record mot1_info("motor info", mot1_speed_f, mot1_pos_f, mot1_volts_f);

VDP::Channel chan0(mot1_info, 0);

std::vector<Initialization> inits = {};

LegacyScreen::LegacyPage init_page, match_page;
Initializer initializer([]() {
  // Initialization code here
  VDB::Device debug_board(vex::PORT1, 9600, chan0);

  while (true) {
    mot1_speed = mot1.velocity(vex::velocityUnits::rpm);
    mot1_pos = mot1.position(vex::rotationUnits::deg);
    mot1.spin(vex::directionType::fwd, mot1_volts, vex::voltageUnits::volt);
    debug_board.send_channel(0);
    vexDelay(100);
    printf("mot1 volts %f\n", mot1_volts);
  }
});
