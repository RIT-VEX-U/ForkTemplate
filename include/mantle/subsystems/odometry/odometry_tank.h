#pragma once

#include "mantle/motor.h"
#include "mantle/sensor.h"
#include "mantle/subsystems/custom_encoder.h"
#include "mantle/subsystems/odometry/odometry_base.h"
#include "core/geometry/rect.h"
#include "core/filter/moving_average.h"
#include "mantle/specs.h"

/**
 * OdometryTank defines an odometry system for a tank drivetrain.
 * This requires encoders in the same orientation as the drive wheels.
 * Odometry is a "start and forget" subsystem, which means once it's created and configured,
 * it will constantly run in the background and track the robot's X, Y and rotation coordinates.
 * ZERO VEX dependencies.
 */
class OdometryTank : public OdometryBase {
  public:
    /**
     * Initialize the Odometry module, calculating position from the drive motors.
     * @param left_side The left motors
     * @param right_side The right motors
     * @param config the specifications that supply the odometry with descriptions of the robot.
     * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
     * @param is_async If true, position will be updated in the background continuously.
     */
    OdometryTank(
      mantle::MotorGroup &left_side, mantle::MotorGroup &right_side, robot_specs_t &config,
      mantle::InertialSensor *imu = nullptr, bool is_async = true
    );

    /**
     * Initialize the Odometry module, calculating position from external encoders.
     * @param left_enc The left encoder
     * @param right_enc The right encoder
     * @param config the specifications that supply the odometry with descriptions of the robot.
     * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
     * @param is_async If true, position will be updated in the background continuously.
     */
    OdometryTank(
      mantle::Encoder &left_enc, mantle::Encoder &right_enc, robot_specs_t &config,
      mantle::InertialSensor *imu = nullptr, bool is_async = true
    );

    /**
     * Update the current position on the field based on the sensors
     * @return the position that odometry has calculated itself to be at
     */
    Pose2d update() override;

    /**
     * set_position tells the odometry to place itself at a position
     * @param newpos the position the odometry will take
     */
    void set_position(const Pose2d &newpos = zero_pos) override;

  private:
    /// Get information from the input hardware and an existing position, and calculate a new current position
    static Pose2d calculate_new_pos(
      robot_specs_t &config, Pose2d &stored_info, double lside_diff, double rside_diff, double angle_deg
    );

    mantle::MotorGroup *left_side = nullptr;
    mantle::MotorGroup *right_side = nullptr;
    mantle::Encoder *left_enc = nullptr;
    mantle::Encoder *right_enc = nullptr;
    mantle::InertialSensor *imu = nullptr;
    robot_specs_t &config;

    double rotation_offset = 0;
    ExponentialMovingAverage ema = ExponentialMovingAverage(3);
};
