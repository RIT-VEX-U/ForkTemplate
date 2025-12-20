#include "core/subsystems/odometry/swerve_pod.h"
#include "core/subsystems/odometry/odometry_base.h"
#include "core/robot_specs.h"
#include "core/utils/controls/feedback_base.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include <vector>

class SwerveDrive{
public:
  enum class BrakeType {
    None,         ///< just send 0 volts to the motors
    Smart,        ///< bring the robot to rest and once it's stopped, try to hold that position
    PreventPushing, ///< tries to make the robot difficult to move by aiming the wheels to the center of the bot
  };
  SwerveDrive(std::vector<SwervePod> &swerve_pods, robot_specs_t &config, OdometryBase *odom);

  void drive_swerve(Translation2d x_y, double rotation_power, double power, BrakeType bt);

  void drive_swerve_raw(Translation2d x_y, double rotation_power);

private:
  std::vector<SwervePod> &swerve_pods;

  Feedback *x_feedback = NULL; ///< feedback to use to drive if none is specified
  Feedback *y_feedback = NULL;  ///< feedback to use to turn if none is specified
  Feedback *turn_feedback = NULL;  ///< feedback to use to turn if none is specified
  PID correction_pid;

  OdometryBase *odom;
  robot_specs_t config;
};
