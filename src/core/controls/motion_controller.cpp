#include "core/controls/motion_controller.h"
#include "core/math/math_util.h"
#include <vector>

/**
 * @brief Construct a new Motion Controller object
 *
 * @param config The definition of how the robot is able to move
 *    max_v Maximum velocity the movement is capable of
 *    accel Acceleration / deceleration of the movement
 *    pid_cfg Definitions of kP, kI, and kD
 *    ff_cfg Definitions of kS, kV, and kA
 */
MotionController::MotionController(m_profile_cfg_t &config)
    : config(config), pid(config.pid_cfg), ff(config.ff_cfg), profile(0, 0, config.max_v, config.accel, config.accel) {}

/**
 * @brief Initialize the motion profile for a new movement
 * This will also reset the PID and profile timers.
 * @param start_pt Movement starting position
 * @param end_pt Movement ending position
 */
void MotionController::init(double start_pt, double end_pt) {
    profile = TrapezoidProfile(start_pt, end_pt, config.max_v, config.accel, config.accel);
    pid.reset();
    tmr.reset();
}

/**
 * @brief Update the motion profile with a new sensor value
 *
 * @param sensor_val Value from the sensor
 * @return the motor input generated from the motion profile
 */
double MotionController::update(double sensor_val) {
    cur_motion = profile.calculate(tmr.value());
    pid.set_target(cur_motion.pos);
    pid.update(sensor_val, cur_motion.vel);

    out = pid.get() + ff.calculate(cur_motion.vel, cur_motion.acc, pid.get());

    if (lower_limit != upper_limit)
        out = clamp(out, lower_limit, upper_limit);

    return out;
}

/// @return the last saved result from the feedback controller
double MotionController::get() { return out; }

/**
 * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
 *
 * @param lower Upper limit
 * @param upper Lower limit
 */
void MotionController::set_limits(double lower, double upper) {
    lower_limit = lower;
    upper_limit = upper;
}

/**
 * @return Whether or not the movement has finished, and the PID
 * confirms it is on target
 */
bool MotionController::is_on_target() {
    return (tmr.value() > profile.total_time()) && pid.is_on_target();
}

/// @return The current position, velocity and acceleration setpoints
motion_t MotionController::get_motion() const { return cur_motion; }
