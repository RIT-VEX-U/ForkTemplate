#include "mantle/subsystems/odometry/odometry_3wheel.h"
#include "core/utils/time.h"
#include "core/math/math_util.h"
#include <cmath>
#include <cstdio>

Odometry3Wheel::Odometry3Wheel(
  mantle::Encoder &lside_fwd, mantle::Encoder &rside_fwd, mantle::Encoder &off_axis, odometry3wheel_cfg_t &cfg, bool is_async
)
    : OdometryBase(is_async), lside_fwd(lside_fwd), rside_fwd(rside_fwd), off_axis(off_axis), cfg(cfg) {}

Pose2d Odometry3Wheel::update() {
    static double lside_old = 0, rside_old = 0, offax_old = 0;

    double lside = lside_fwd.position(core::RotationUnits::Degrees);
    double rside = rside_fwd.position(core::RotationUnits::Degrees);
    double offax = off_axis.position(core::RotationUnits::Degrees);

    double lside_delta = lside - lside_old;
    double rside_delta = rside - rside_old;
    double offax_delta = offax - offax_old;

    lside_old = lside;
    rside_old = rside;
    offax_old = offax;

    Pose2d updated_pos = calculate_new_pos(lside_delta, rside_delta, offax_delta, current_pos, cfg);

    static Pose2d last_pos = updated_pos;
    static double last_speed = 0;
    static double last_ang_speed = 0;
    static core::Timer tmr;

    double speed_local = 0;
    double accel_local = 0;
    double ang_speed_local = 0;
    double ang_accel_local = 0;
    bool update_vel_accel = tmr.time_sec() > 0.1;

    // This loop runs too fast. Only check at LEAST every 1/10th sec
    if (update_vel_accel) {
        double elapsed = tmr.time_sec();
        if (elapsed <= 0) elapsed = 0.001;

        // Calculate robot velocity
        speed_local = updated_pos.translation().distance(last_pos.translation()).to(units::in) / elapsed;

        // Calculate robot acceleration
        accel_local = (speed_local - last_speed) / elapsed;

        // Calculate robot angular velocity (deg/sec)
        ang_speed_local =
          smallest_angle(updated_pos.rotation().degrees(), last_pos.rotation().degrees()) / elapsed;

        // Calculate robot angular acceleration (deg/sec^2)
        ang_accel_local = (ang_speed_local - last_ang_speed) / elapsed;

        tmr.reset();
        last_pos = updated_pos;
        last_speed = speed_local;
        last_ang_speed = ang_speed_local;
    }

    this->current_pos = updated_pos;
    if (update_vel_accel) {
        this->speed = speed_local;
        this->accel = accel_local;
        this->ang_speed_deg = ang_speed_local;
        this->ang_accel_deg = ang_accel_local;
    }

    return current_pos;
}

Pose2d Odometry3Wheel::calculate_new_pos(
  double lside_delta_deg, double rside_delta_deg, double offax_delta_deg, Pose2d old_pos, odometry3wheel_cfg_t cfg
) {
    Pose2d retval;

    // Arclength formula for encoder degrees -> single wheel distance driven
    double lside_dist = (cfg.wheel_diam / 2.0) * Rotation2d::deg2rad(lside_delta_deg);
    double rside_dist = (cfg.wheel_diam / 2.0) * Rotation2d::deg2rad(rside_delta_deg);
    double offax_dist = (cfg.wheel_diam / 2.0) * Rotation2d::deg2rad(offax_delta_deg);

    // Inverse arclength formula for arc distance driven -> robot angle
    double delta_angle_rad = (rside_dist - lside_dist) / cfg.wheelbase_dist;

    // Distance along the robot's local Y axis (forward/backward)
    double dist_local_y = (lside_dist + rside_dist) / 2.0;

    // Distance along the robot's local X axis (right/left)
    double dist_local_x = offax_dist - (delta_angle_rad * cfg.off_axis_center_dist);

    // Change in displacement as a vector, on the local coordinate system (+y = robot fwd)
    Translation2d local_displacement(units::Length(dist_local_x, units::in), units::Length(dist_local_y, units::in));

    // Rotate the local displacement to match the old robot's rotation
    double dir_delta_from_trans_rad = local_displacement.theta().radians() - (PI / 2.0);
    double global_dir_rad = wrap_angle_rad(dir_delta_from_trans_rad + old_pos.rotation().radians());
    Translation2d global_displacement(local_displacement.norm(), Rotation2d(global_dir_rad));

    // Tack on the position change to the old position
    Translation2d new_pos_vec = old_pos.translation() + global_displacement;

    retval = Pose2d(new_pos_vec.x(), new_pos_vec.y(), wrap_angle_rad(old_pos.rotation().radians() + delta_angle_rad));

    return retval;
}

void Odometry3Wheel::tune(mantle::Controller &con, TankDrive &drive) {
    // STEP 1: Align robot and reset odometry
    con.Screen.clearScreen();
    con.Screen.setCursor(1, 1);
    con.Screen.print("Wheel Diameter Test");
    con.Screen.newLine();
    con.Screen.print("Align robot with ref");
    con.Screen.newLine();
    con.Screen.newLine();
    con.Screen.print("Press A to continue");
    while (!con.ButtonA.pressing()) {
        core::delay_ms(20);
    }

    double old_lval = lside_fwd.position(core::RotationUnits::Degrees);
    double old_rval = rside_fwd.position(core::RotationUnits::Degrees);

    // Step 2: Drive robot a known distance
    con.Screen.clearLine(2);
    con.Screen.setCursor(2, 1);
    con.Screen.print("Drive or Push robot");
    con.Screen.newLine();
    con.Screen.print("10 feet (5 tiles)");
    con.Screen.newLine();
    con.Screen.print("Press A to continue");
    while (!con.ButtonA.pressing()) {
        drive.drive_arcade(con.Axis3.position() / 100.0, con.Axis1.position() / 100.0);
        core::delay_ms(20);
    }

    // Wheel diameter is ratio of expected distance / measured distance
    double avg_deg = ((lside_fwd.position(core::RotationUnits::Degrees) - old_lval) + (rside_fwd.position(core::RotationUnits::Degrees) - old_rval)) / 2.0;
    double measured_dist = 0.5 * Rotation2d::deg2rad(avg_deg); // Simulate diam=1", radius=1/2"
    double found_diam = 120.0 / measured_dist;

    // Step 3: Reset alignment for turning test
    con.Screen.clearScreen();
    con.Screen.setCursor(1, 1);
    con.Screen.print("Wheelbase Test");
    con.Screen.newLine();
    con.Screen.print("Align robot with ref");
    con.Screen.newLine();
    con.Screen.newLine();
    con.Screen.print("Press A to continue");
    while (!con.ButtonA.pressing()) {
        core::delay_ms(20);
    }
    con.Screen.clearScreen();

    old_lval = lside_fwd.position(core::RotationUnits::Degrees);
    old_rval = rside_fwd.position(core::RotationUnits::Degrees);
    double old_offax = off_axis.position(core::RotationUnits::Degrees);

    con.Screen.setCursor(2, 1);
    con.Screen.clearLine();
    con.Screen.print("Turn robot 10");
    con.Screen.newLine();
    con.Screen.print("times in place");
    con.Screen.newLine();
    con.Screen.print("Press A to continue");
    while (!con.ButtonA.pressing()) {
        drive.drive_arcade(0, con.Axis1.position() / 100.0);
        core::delay_ms(20);
    }

    double lside_dist = Rotation2d::deg2rad(lside_fwd.position(core::RotationUnits::Degrees) - old_lval) * (found_diam / 2.0);
    double rside_dist = Rotation2d::deg2rad(rside_fwd.position(core::RotationUnits::Degrees) - old_rval) * (found_diam / 2.0);
    double offax_dist = Rotation2d::deg2rad(off_axis.position(core::RotationUnits::Degrees) - old_offax) * (found_diam / 2.0);

    double expected_angle = 10 * (2 * PI);
    double found_wheelbase = std::fabs(rside_dist - lside_dist) / expected_angle;
    double found_offax_center_dist = offax_dist / expected_angle;

    con.Screen.clearScreen();
    con.Screen.setCursor(1, 1);
    con.Screen.print("Diam: %f", found_diam);
    con.Screen.newLine();
    con.Screen.print("whlbase: %f", found_wheelbase);
    con.Screen.newLine();
    con.Screen.print("offax: %f", found_offax_center_dist);

    printf(
      "Tuning completed.\n  Wheel Diameter: %f\n  Wheelbase: %f\n  Off Axis Distance: %f\n", found_diam,
      found_wheelbase, found_offax_center_dist
    );
}
