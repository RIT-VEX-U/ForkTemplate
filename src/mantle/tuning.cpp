#include "mantle/tuning.h"
#include "core/filter/moving_average.h"
#include "core/math/math_util.h"
#include "core/utils/time.h"
#include <cmath>
#include <vector>

namespace mantle {

FeedForward::ff_config_t tune_feedforward(mantle::MotorGroup &motor, double pct, double duration) {
    FeedForward::ff_config_t out = {};

    double start_pos = motor.position(core::RotationUnits::Rotations);

    // ========== kS Tuning =========
    // Start at 0 and slowly increase the power until the robot starts moving
    double power = 0;
    while (std::fabs(motor.position(core::RotationUnits::Rotations) - start_pos) < 0.05) {
        motor.spin(core::Direction::Forward, power, core::VoltageUnits::Volt);
        power += 0.001;
        core::delay_ms(100);
    }
    out.kS = power;
    motor.stop();

    // ========== kV / kA Tuning =========

    std::vector<std::pair<double, double>> vel_data_points;   // time, velocity
    std::vector<std::pair<double, double>> accel_data_points; // time, accel

    double max_speed = 0;
    core::Timer tmr;
    double time = 0;

    MovingAverage vel_ma(3);
    MovingAverage accel_ma(3);

    do {
        double last_time = time;
        time = tmr.time_sec();
        double dt = time - last_time;

        vel_ma.add_entry(motor.velocity(core::VelocityUnits::Rpm));
        accel_ma.add_entry(motor.velocity(core::VelocityUnits::Rpm) / (dt > 0 ? dt : 0.01));

        double speed = vel_ma.get_value();
        double accel = accel_ma.get_value();

        if (speed > max_speed) {
            max_speed = speed;
        }

        if (time > 0.25) {
            vel_data_points.push_back(std::pair<double, double>(time, speed));
            accel_data_points.push_back(std::pair<double, double>(time, accel));
        }

        core::delay_ms(10);
    } while (time < duration);

    motor.stop();

    // Calculate kV (volts/12 per unit per second)
    if (max_speed > 0) {
        out.kV = (pct - out.kS) / max_speed;
    }

    // Calculate kA (volts/12 per unit per second^2)
    std::vector<std::pair<double, double>> accel_per_pct;
    for (size_t i = 0; i < vel_data_points.size(); i++) {
        accel_per_pct.push_back(std::pair<double, double>(
            pct - out.kS - (vel_data_points[i].second * out.kV),
            accel_data_points[i].second
        ));
    }

    if (!accel_per_pct.empty()) {
        double regres_slope = calculate_linear_regression(accel_per_pct).first;
        if (regres_slope != 0) {
            out.kA = 1.0 / regres_slope;
        }
    }

    return out;
}

FeedForward::ff_config_t tune_feedforward(TankDrive &drive, OdometryTank &odometry, double pct, double duration) {
    FeedForward::ff_config_t out = {};

    Pose2d start_pos = odometry.get_position();

    // ========== kS Tuning =========
    double power = 0;
    while (start_pos.translation().distance(odometry.get_position().translation()) < units::Length(0.05, units::in)) {
        drive.drive_tank(power, power, 1);
        power += 0.001;
        core::delay_ms(100);
    }
    out.kS = power;
    drive.stop();

    // ========== kV / kA Tuning =========
    std::vector<std::pair<double, double>> vel_data_points;
    std::vector<std::pair<double, double>> accel_data_points;

    double max_speed = 0;
    core::Timer tmr;
    double time = 0;

    MovingAverage vel_ma(3);
    MovingAverage accel_ma(3);

    do {
        time = tmr.time_sec();

        vel_ma.add_entry(odometry.get_speed());
        accel_ma.add_entry(odometry.get_accel());

        double speed = vel_ma.get_value();
        double accel = accel_ma.get_value();

        if (speed > max_speed)
            max_speed = speed;

        if (time > 0.25) {
            vel_data_points.push_back(std::pair<double, double>(time, speed));
            accel_data_points.push_back(std::pair<double, double>(time, accel));
        }

        core::delay_ms(10);
    } while (time < duration);

    drive.stop();

    if (max_speed > 0) {
        out.kV = (pct - out.kS) / max_speed;
    }

    std::vector<std::pair<double, double>> accel_per_pct;
    for (size_t i = 0; i < vel_data_points.size(); i++) {
        accel_per_pct.push_back(std::pair<double, double>(
            pct - out.kS - (vel_data_points[i].second * out.kV),
            accel_data_points[i].second
        ));
    }

    if (!accel_per_pct.empty()) {
        double regres_slope = calculate_linear_regression(accel_per_pct).first;
        if (regres_slope != 0) {
            out.kA = 1.0 / regres_slope;
        }
    }

    return out;
}

} // namespace mantle
