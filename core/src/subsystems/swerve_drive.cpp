#include "core/subsystems/swerve_drive.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/math/geometry/twist2d.h"

SwerveDrive::SwerveDrive(std::vector<SwervePod> &swerve_pods, robot_specs_t &config, OdometryBase *odom)
    : swerve_pods(swerve_pods), x_feedback(config.x_feedback), y_feedback(config.y_feedback), turn_feedback(config.turn_feedback), correction_pid(config.correction_pid),  odom(odom),
      config(config) {
}

void SwerveDrive::drive_swerve(Translation2d x_y, double rotation_power, double power = 1, BrakeType bt = BrakeType::None) {

  x_y.normalize(power);

  // rotation_power *= power;

  double brake_threshold = 1;
  bool should_brake = fabs(x_y.norm()) < brake_threshold && fabs(rotation_power) < brake_threshold;

  bool captured_position = false;
  bool was_breaking = false;

  if (!should_brake) {
    drive_swerve_raw(x_y, rotation_power);
    was_breaking = false;
    return;
  }
  if (should_brake && !was_breaking) {
    captured_position = false;
  }

  if (bt == BrakeType::Smart) {
    static Pose2d target_pose(0.0, 0.0, 0.0);

    double vel = odom->get_speed();
    if (fabs(vel) <= 0.01 && !captured_position) {
      target_pose = odom->get_position();
      captured_position = true;
    } else if (captured_position) {
      double dist_to_target = target_pose.translation().distance(odom->get_position().translation());
      if (dist_to_target < 12.0) {
        // drive_to_point(target_pose.x(), target_pose.y(), vex::fwd);
      } else {
        target_pose = odom->get_position();
        // reset_auto();
      }
    } 
  }
  else if(bt == BrakeType::PreventPushing){
    for(SwervePod pod : swerve_pods){
      pod.move_swerve(pod.get_location().theta(), 0, true);
    }
  }
  else if(bt == BrakeType::None){
    for(SwervePod pod : swerve_pods){
      pod.move_swerve(pod.get_rotation(), 0, true);
    }
  }
  was_breaking = should_brake;
}

void SwerveDrive::drive_swerve_raw(Translation2d x_y, double rotation_power) {
  Translation2d center_of_rotation{0,0};
  for(SwervePod pod: swerve_pods){
    center_of_rotation = center_of_rotation + pod.get_location();
  }
  printf("rot power: %.2f\n", rotation_power);
  //use in odometry later maybe
  // double Vx = 0;
  // double Vy = 0;
  // double V_angular = 0;
  // for(SwervePod pod: swerve_pods){
  //   Vx += pod.get_tangential_vel() * pod.get_rotation().f_cos();
  //   Vy += pod.get_tangential_vel() * pod.get_rotation().f_sin();
  //   V_angular += pod.get_angular_vel(center_of_rotation);
  // }
  // Twist2d the_twist(Vx, Vy, V_angular);

  for(SwervePod pod : swerve_pods){
    Translation2d to_center = pod.get_location() - center_of_rotation;
    double Vx = x_y.x() - to_center.y() * rotation_power;
    double Vy = x_y.y() + to_center.x() * rotation_power;
    Translation2d pod_movement_vector(Vx, Vy);
    pod.move_swerve(pod_movement_vector.theta(), pod_movement_vector.norm(), true);
  }
}
