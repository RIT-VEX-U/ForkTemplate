#include "mantle/subsystems/odometry/odometry_tank.h"
#include "core/utils/time.h"
#include "core/math/math_util.h"
#include <cmath>

OdometryTank::OdometryTank(
    mantle::MotorGroup& left_side, mantle::MotorGroup& right_side, robot_specs_t& config,
    mantle::InertialSensor* imu, bool is_async
)
    : OdometryBase(is_async),
      left_side(&left_side),
      right_side(&right_side),
      left_enc(nullptr),
      right_enc(nullptr),
      imu(imu),
      config(config) {}

OdometryTank::OdometryTank(
    mantle::Encoder& left_enc, mantle::Encoder& right_enc, robot_specs_t& config,
    mantle::InertialSensor* imu, bool is_async
)
    : OdometryBase(is_async),
      left_side(nullptr),
      right_side(nullptr),
      left_enc(&left_enc),
      right_enc(&right_enc),
      imu(imu),
      config(config) {}

void OdometryTank::set_position(const Pose2d& newpos) {
    mut.lock();
    rotation_offset =
            newpos.rotation().degrees() - (current_pos.rotation().degrees() - rotation_offset);
    mut.unlock();

    OdometryBase::set_position(newpos);
}

Pose2d OdometryTank::update() {
    double lside_revs = 0, rside_revs = 0;

    if (left_side != nullptr && right_side != nullptr) {
        lside_revs = left_side->position(core::RotationUnits::Rotations) / config.odom_gear_ratio;
        rside_revs = right_side->position(core::RotationUnits::Rotations) / config.odom_gear_ratio;
    } else if (left_enc != nullptr && right_enc != nullptr) {
        lside_revs = left_enc->position(core::RotationUnits::Rotations) / config.odom_gear_ratio;
        rside_revs = right_enc->position(core::RotationUnits::Rotations) / config.odom_gear_ratio;
    }

    double angle = 0;

    // If the IMU data was passed in, use it for rotational data
    if (imu == nullptr || !imu->is_installed() || imu->is_calibrating()) {
        double distance_diff = (rside_revs - lside_revs) * PI * config.odom_wheel_diam;
        angle = ((180.0 / PI) * (distance_diff / config.dist_between_wheels));
    } else {
        // Translate "clockwise positive" to "CCW negative"
        angle = -imu->rotation(core::RotationUnits::Degrees);
    }

    // Offset the angle, if we've done a set_position
    angle += rotation_offset;

    /*
     * Limit the angle between 0 and 360.
     * fmod (floating-point modulo) gets it between -359 and +359, so tack on another 360 if it's
     * negative.
     */
    angle = std::fmod(angle, 360.0);
    if (angle < 0) {
        angle += 360.0;
    }

    current_pos = calculate_new_pos(config, current_pos, lside_revs, rside_revs, angle);

    static Pose2d last_pos = current_pos;
    static double last_speed = 0;
    static double last_ang_speed = 0;
    static core::Timer tmr;
    bool update_vel_accel = tmr.time_sec() > 0.02;

    // This loop runs too fast. Only check at LEAST every 1/10th sec
    if (update_vel_accel) {
        // Calculate robot velocity
        double elapsed = tmr.time_sec();
        if (elapsed <= 0) elapsed = 0.001;

        double this_speed =
                current_pos.translation().distance(last_pos.translation()).to(units::in) / elapsed;
        ema.add_entry(this_speed);
        speed = ema.get_value();
        // Calculate robot acceleration
        accel = (speed - last_speed) / elapsed;

        // Calculate robot angular velocity (deg/sec)
        ang_speed_deg =
                smallest_angle(current_pos.rotation().degrees(), last_pos.rotation().degrees()) / elapsed;

        // Calculate robot angular acceleration (deg/sec^2)
        ang_accel_deg = (ang_speed_deg - last_ang_speed) / elapsed;

        tmr.reset();
        last_pos = current_pos;
        last_speed = speed;
        last_ang_speed = ang_speed_deg;
    }

    return current_pos;
}

Pose2d OdometryTank::calculate_new_pos(
        robot_specs_t& config, Pose2d& curr_pos, double lside_revs, double rside_revs,
        double angle_deg
) {
    Pose2d new_pos;

    static double stored_lside_revs = lside_revs;
    static double stored_rside_revs = rside_revs;

    double lside_diff = (lside_revs - stored_lside_revs) * PI * config.odom_wheel_diam;
    double rside_diff = (rside_revs - stored_rside_revs) * PI * config.odom_wheel_diam;
    double dist_driven = (lside_diff + rside_diff) / 2.0;

    double angle = angle_deg * PI / 180.0;  // Degrees to radians

    Translation2d chg_point(units::Length(dist_driven, units::in), Rotation2d(angle));
    Translation2d curr_point(curr_pos.x(), curr_pos.y());

    Translation2d new_point = curr_point + chg_point;
    new_pos = Pose2d(new_point, Rotation2d(units::Angle(angle_deg, units::degrees)));

    stored_lside_revs = lside_revs;
    stored_rside_revs = rside_revs;

    return new_pos;
}
