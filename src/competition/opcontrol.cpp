#include "competition/opcontrol.h"
#include <v5_apiuser.h>
#include <vex_motorgroup.h>
#include "core/subsystems/tank_drive.h"

//define the controller
vex::controller Con;

//define a motor at a port, with a specific gearsetting (the color gearbox) and if it is reversed or not
vex::motor motor1(PORT1, vex::gearSetting::ratio6_1, false);

//set up driving
// define the motors for driving
vex::motor left1(PORT1, vex::gearSetting::ratio6_1, false);
vex::motor left2(PORT1, vex::gearSetting::ratio6_1, false);
vex::motor left3(PORT1, vex::gearSetting::ratio6_1, false);

vex::motor right1(PORT1, vex::gearSetting::ratio6_1, false);
vex::motor right2(PORT1, vex::gearSetting::ratio6_1, false);
vex::motor right3(PORT1, vex::gearSetting::ratio6_1, false);
//add those motors to a motor group (just list the ones you created for each side)
vex::motor_group left_motors(left1, left2, left3);
vex::motor_group right_motors(right1, right2, right3);

//define the configurations for the robot (dont need to worry about this unless working on autos)
robot_specs_t config{};

//add the left motors, right motors, and config to the drive system.
TankDrive drive_sys(left_motors, right_motors, config);

void opcontrol() {

  //do something when a button is pressed
  Con.ButtonR1.pressed([](){
    //spin the motor forward at 100% speed
    motor1.spin(vex::forward, 100, vex::velocityUnits::pct);
  });

  //the bot is contantly running this loop
  while(true){
    //check if the button is no longer being pressed
    if(Con.ButtonR1.pressing() == false){
      //stop the motors
      motor1.stop();
    }
    //get the position of the controller joystick axis(look at the controller for the number)
    // it gives us a number from -100 to 100 but we use from -1 to 1 so divide it by 100
    // this is for arcade drive
    double left = Con.Axis3.position() / 100;
    double right = Con.Axis1.position() / 100;

    drive_sys.drive_arcade(left, right);
    //this is the same thing but for tank drive (dont use both at the same time)
    // double left = Con.Axis3.position() / 100;
    // double right = Con.Axis2.position() / 100;

    // drive_sys.drive_tank(left, right);

    //delay for 100ms so we dont overwhelm the robot
    vexDelay(100);
  }
}
