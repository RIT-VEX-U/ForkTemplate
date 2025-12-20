#include "competition/opcontrol.h"

#include "core/robot_specs.h"
#include "core/subsystems/odometry/odometry_base.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "robot-config.h"
#include "core/subsystems/swerve_drive.h"

PID::pid_config_t rotation_config{
  .p = 0.8,
  .d = 0.04,
  .deadband = 2,
  .on_target_time = 0.1,
};

PID rotation_pid(rotation_config);

SwervePod pod1(front_left_top, front_left_bottom, {4, -6.625}, rotation_pid, 0.216, 1.225, 2.75);
SwervePod pod2(front_right_top, front_right_bottom, {4, 6.625}, rotation_pid, 0.216, 1.225, 2.75);
SwervePod pod3(back_left_top, back_left_bottom, {-4,-6.625}, rotation_pid, 0.216, 1.225, 2.75);
SwervePod pod4(back_right_top, back_right_bottom, {-4,6.625}, rotation_pid, 0.216, 1.225, 2.75);
std::vector<SwervePod> swerve_pods{pod2, pod3, pod4};
robot_specs_t config{
};


SwerveDrive swerve_drive(swerve_pods, config, NULL);


void opcontrol() {
  while(true){
    Translation2d Con_trans(Con.Axis4.position(), Con.Axis3.position());
    if(Con_trans.norm() < 15){
      Con_trans.normalize(0);
    }
    swerve_drive.drive_swerve(Con_trans, Con.Axis1.position(), 0.5, SwerveDrive::BrakeType::PreventPushing);

    // swerve_drive.drive_swerve_raw(Con_trans, Con.Axis1.position());

    vexDelay(50);
  }
}
