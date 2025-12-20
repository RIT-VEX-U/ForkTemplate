#include "competition/opcontrol.h"

//define the controller
vex::controller Con;

//define a motor at a port, with a specific gearsetting (the color gearbox) and if it is reversed or not
vex::motor motor1(PORT1, vex::gearSetting::ratio6_1, false);

void opcontrol() {

  //do something when a button is pressed
  Con.ButtonR1.pressed([](){
    //spin the motor forward at 100% speed
    motor1.spin(vex::forward, 100, vex::velocityUnits::pct);
  });

  while(true){
    //check if the button is no longer being pressed
    if(Con.ButtonR1.pressing() == false){
      //stop the motors
      motor1.stop();
    }
  }
}
