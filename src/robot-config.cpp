#include "robot-config.h"
#include "competition/autonomous.h"
#include "competition/opcontrol.h"


vex::brain brain;
vex::competition competition;
vex::controller controller;


vex::motor mot1(vex::PORT1);
vex::motor mot2(vex::PORT2);

int mot1_speed = mot1.velocity(vex::velocityUnits::rpm);
int mot1_pos = mot1.position(vex::rotationUnits::rev);

VDP::Field mot1_speed_f("mot1 speed", mot1_speed);
VDP::Field mot1_pos_f("mot1 position", mot1_pos);

VDP::Record mot1_info("motor info", mot1_speed_f, mot1_pos_f);

VDP::Channel chan0(mot1_info, 0);

int mot2_speed = 40;
int mot2_pos = 20;

VDP::Field mot2_speed_f("mot1 speed", mot1_speed);
VDP::Field mot2_pos_f("mot1 position", mot1_pos);

VDP::Record mot2_info("motor info", mot2_speed_f, mot2_pos_f);
std::vector<Initialization> inits = {};

LegacyScreen::LegacyPage init_page, match_page;
Initializer initializer(inits, LegacyScreen::InitializerPage::timed_selector(20),
LegacyScreen::pre_initialize(brain, initializer, &init_page), []() {
    // Initialization code here
  printf("%s", mot1_info.data_to_string().c_str());
  mot1_speed = 55;

  printf("%s", mot1_info.data_to_string().c_str());

  VDP::Packet mot2_serialized = mot2_info.serialize_data();

  for(uint8_t byte: mot2_serialized) {
    printf("%d\n", byte);
  }

  mot1_info.apply_update(mot2_serialized);

  VDB::Device debug_board(vex::PORT1, 9600, chan0);

  printf("%s", mot1_info.data_to_string().c_str());

  while (true) {
    debug_board.send_channel(0);
  }
});
