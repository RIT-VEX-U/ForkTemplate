#pragma once

#include "core/controls/feedforward.h"
#include "mantle/motor.h"
#include "mantle/subsystems/tank_drive.h"
#include "mantle/subsystems/odometry/odometry_tank.h"

namespace mantle {

/**
 * Characterize a motor group and find the feedforward configuration parameters.
 * @param motor The motor group to test
 * @param pct Maximum velocity in percent (0.0 - 1.0)
 * @param duration Duration of the test movement in seconds
 * @return Tuned feedforward configuration (kS, kV, kA)
 */
FeedForward::ff_config_t tune_feedforward(mantle::MotorGroup &motor, double pct, double duration);


/**
 * Characterize a robot's drivetrain and automatically tune feedforward.
 * @param drive The tank drive subsystem
 * @param odometry The tank odometry subsystem
 * @param pct Maximum velocity in percent (0.0 - 1.0)
 * @param duration Duration of the test movement in seconds
 * @return Tuned feedforward configuration (kS, kV, kA)
 */
FeedForward::ff_config_t tune_feedforward(TankDrive &drive, OdometryTank &odometry, double pct = 0.6, double duration = 2.0);

} // namespace mantle
