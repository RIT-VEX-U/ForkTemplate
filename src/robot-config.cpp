#include "robot-config.h"
void robot_init() {

    vex::brain Brain;
    vex::controller con;

    //MOTORS
    vex::motor front_left(PORT1, vex::gearSetting::ratio6_1, true);
    vex::motor back_left(PORT3, vex::gearSetting::ratio6_1, true);
    vex::motor middle_left(PORT2, vex::gearSetting::ratio6_1, true);
    vex::motor_group left_motors(front_left, back_left, middle_left);

    vex::motor back_right(PORT6, vex::gearSetting::ratio6_1, false);
    vex::motor middle_right(PORT7, vex::gearSetting::ratio6_1, false);
    vex::motor front_right(PORT8, vex::gearSetting::ratio6_1, false);
    vex::motor_group right_motors(back_right, middle_right, front_right);
    //SENSORS
    vex::inertial imu(PORT10);
    PID::pid_config_t drive_pid_cfg{
        .p = 0.0,
        .i = 0.0,
        .d = 0.0,
        .deadband = 0.5,
        .on_target_time = 0.1,
        .error_method = PID::ERROR_TYPE::LINEAR,
    };

    PID drive_pid{drive_pid_cfg};

    PID::pid_config_t turn_pid_cfg{
        .p = 0.0,
        .i = 0.0,
        .d = 0.0,
        .deadband = 1,
        .on_target_time = 0.1,
        .error_method = PID::ERROR_TYPE::ANGULAR,
    };

    PID turn_pid{turn_pid_cfg};

    PID::pid_config_t correction_pid_cfg{
        .p = 0.0,
        .i = 0.0,
        .d = 0.0,
        .deadband = 0.5,
        .on_target_time = 0.1,
        .error_method = PID::ERROR_TYPE::ANGULAR,
    };

    PID correction_pid{correction_pid_cfg};

    robot_specs_t robot_cfg{
        .robot_radius = 10,
        .correction_pid = correction_pid_cfg,
        .drive_feedback = &drive_pid,
        .turn_feedback = &turn_pid,
        .dist_between_wheels = 12.25,
        .odom_gear_ratio = 4/3,
    };
    OdometryTank odom(left_motors, right_motors, robot_cfg, &imu);

    TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

    

}

