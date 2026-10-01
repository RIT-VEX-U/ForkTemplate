#pragma once

#include "mantle/comm/serial.h"
#include "mantle/subsystems/odometry/odometry_serial.h"
#include "vex.h"


namespace crust {

/**
 * Concrete VEX V5 Smart Port Generic Serial driver implementing mantle::SerialPort.
 */
class V5SerialPort : public mantle::SerialPort {
private:
    int32_t port;

public:
    V5SerialPort(int32_t port, int32_t baudrate) : port(port) {
        vexGenericSerialEnable(port, 0);
        vexGenericSerialBaudrate(port, baudrate);
    }

    void write(const uint8_t *data, size_t length) override {
        vexGenericSerialTransmit(port, const_cast<uint8_t*>(data), length);
    }

    int read_char() override {
        return vexGenericSerialReadChar(port);
    }

    int available() override {
        return vexGenericSerialReceiveAvail(port);
    }

    int32_t get_port() const { return port; }
};

/**
 * Concrete V5 OdometrySerial tying V5SerialPort directly into OdometrySerial.
 */
class V5OdometrySerial : public OdometrySerial {
private:
    V5SerialPort serial_dev;

public:
    V5OdometrySerial(
        bool is_async, bool calc_vel_acc_on_brain, Pose2d initial_pose, Pose2d sensor_offset,
        int32_t port, int32_t baudrate
    )
        : serial_dev(port, baudrate),
          OdometrySerial(serial_dev, is_async, calc_vel_acc_on_brain, initial_pose, sensor_offset) {}
};

} // namespace crust

