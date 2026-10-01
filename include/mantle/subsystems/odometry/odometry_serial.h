#pragma once

#include "mantle/comm/serial.h"
#include "mantle/subsystems/odometry/odometry_base.h"
#include "core/geometry/pose2d.h"

/**
 * OdometrySerial
 *
 * This class handles the code for an odometry setup where calculations are done on an external coprocessor.
 * Data is sent to the brain via smart port, using an abstract serial (UART) connection.
 * ZERO VEX dependencies.
 *
 * @author Jack Cammarata
 * @date Jan 16 2025
 */
class OdometrySerial : public OdometryBase {
  public:
    /// Construct a new Odometry Serial Object
    OdometrySerial(
      mantle::SerialPort &serial, bool is_async, bool calc_vel_acc_on_brain,
      Pose2d initial_pose = Pose2d(), Pose2d sensor_offset = Pose2d()
    );

    void send_config(const Pose2d &initial_pose, const Pose2d &sensor_offset, const bool &calc_vel_acc_on_brain);

    /**
     * Update the current position of the robot once by reading a single packet from the serial port
     *
     * @return the robot's updated position
     */
    Pose2d update() override;

    /// Resets the position and rotational data to the input.
    void set_position(const Pose2d &new_pose) override;

    int receive_cobs_packet(uint8_t *buffer, size_t buffer_size);

    Pose2d get_position(void) override;

    Pose2d get_pose2d(void);

    size_t cobs_decode(const uint8_t *buffer, size_t length, void *data);

    size_t cobs_encode(const void *data, size_t length, uint8_t *buffer);

    double get_speed() override;

    double get_accel() override;

  private:
    mantle::SerialPort &serial;
    bool calc_vel_acc_on_brain;

    Pose2d pose;
    Pose2d pose_offset;

    double speed = 0;
    double accel = 0;
    double ang_speed_deg = 0;
    double ang_accel_deg = 0;
};