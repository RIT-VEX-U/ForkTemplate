#include "mantle/comm/vdb/builtins.hpp"

#include <cstdint>
#include <string>
#include <utility>

#include "mantle/comm/vdb/protocol.hpp"
#include "mantle/comm/vdb/types.hpp"
#include "mantle/motor.h"
#include "core/utils/time.h"

namespace VDP {
/**
 * Creates a record that contains a
 * Float of a timestamp
 * Part of data
 * @param name the name of the record to create
 * @param data the data to put into the record
 */
TimestampedRecord::TimestampedRecord(std::string name, Part* data)
    : Record(name),
      timestamp(new Float("timestamp(sec)", []() { return (float)core::Timer::system_seconds(); })),
      data(data) {
    Record::set_fields({timestamp, (PartPtr)data});
}

/// sets the data that the Timestamp Parts hold
void TimestampedRecord::fetch() {
    timestamp->fetch();
    data->fetch();
}
/**
 * Creates a record that contains a
 * Float of the motor position
 * Float of the motor velocity
 * Float of the motor tempurature
 * Float of the motor voltage
 * Float of the motor current
 * @param name the name of the record to create
 * @param mot the motor to get data from
 */
MotorDataRecord::MotorDataRecord(std::string name, mantle::Motor& motor)
    : Record(std::move(name)),
      mot(motor),
      pos(new Float("Position(deg)")),
      vel(new Float("velocity(dps)")),
      temp(new Float("Temperature(C)")),
      voltage(new Float("Voltage(V)")),
      current(new Float("Current(%)")) {
    Record::set_fields({pos, vel, temp, voltage, current});
}

/// sets the data that the Motor Parts hold
void MotorDataRecord::fetch() {
    pos->set_value((float)mot.position(core::RotationUnits::Degrees));
    vel->set_value((float)(mot.velocity(core::VelocityUnits::Rpm) * 6.0));
    temp->set_value((float)mot.temperature());
    voltage->set_value((float)mot.voltage());
    current->set_value((float)mot.current());
}
/**
 * Creates a record that contains a
 * Float of the odometry X postion
 * Float of the odometry Y postion
 * Float of the odometry Rotation
 * @param name the name of the record to create
 * @param odom the odometry to get data from
 */
OdometryDataRecord::OdometryDataRecord(std::string name, OdometryBase& odom)
    : Record(std::move(name)),
      odom(odom),
      X(new Float("X")),
      Y(new Float("Y")),
      ROT(new Float("Rotation")) {
    Record::set_fields({X, Y, ROT});
}

/// sets the data that the Odometry Parts hold
void OdometryDataRecord::fetch() {
    X->set_value((float)odom.get_position().x(units::in));
    Y->set_value((float)odom.get_position().y(units::in));
    ROT->set_value((float)odom.get_position().rotation().degrees());
}
/**
 * Creates a record for taking odometry data from the debug board
 * @param name the name of the record to create
 * @param odom the odometry to get data from
 */
OdometryControlRecord::OdometryControlRecord(std::string name, OdometryBase& odom)
    : Record(std::move(name)),
      odom(odom),
      X(new Float("X")),
      Y(new Float("Y")),
      ROT(new Float("Rotation")) {
    Record::set_fields({X, Y, ROT});
}
/// sets the odometry position to the values from the debug board
void OdometryControlRecord::response() {
    odom.set_position(
            {units::Length(X->get_value(), units::in), units::Length(Y->get_value(), units::in),
             ROT->get_value()}
    );
}

/**
 * Creates a record that contains a
 * Float of the pid P value
 * Float of the pid I value
 * Float of the pid D value
 * Float of the pid error
 * Float of the pid output
 * String of the pid type (linear or angular)
 * @param name the name of the record to create
 * @param pid the pid to get data from
 */
PIDDataRecord::PIDDataRecord(std::string name, PID& pid)
    : Record(std::move(name)),
      pid(pid),
      P(new Float("P")),
      I(new Float("I")),
      D(new Float("D")),
      ERROR(new Float("Error")),
      OUTPUT(new Float("Output")),
      TYPE(new String("Type")) {
    Record::set_fields({TYPE, P, I, D, ERROR, OUTPUT});
}

/// sets the data that the PID Parts hold to be sent to the board
void PIDDataRecord::fetch() {
    P->set_value((float)pid.config.p);
    I->set_value((float)pid.config.i);
    D->set_value((float)pid.config.d);
    ERROR->set_value((float)pid.get_error());
    OUTPUT->set_value((float)pid.get_output());
    if (pid.config.error_method == PID::ANGULAR) {
        TYPE->set_value("Angular");
    } else {
        TYPE->set_value("Linear");
    }
}

/// Defines a record for setting pid values from the board
PIDControlRecord::PIDControlRecord(std::string name, PID& pid)
    : Record(std::move(name)), pid(pid), P(new Float("P")), I(new Float("I")), D(new Float("D")) {
    Record::set_fields({P, I, D});
}

/// sets the PID values to the values from the board
void PIDControlRecord::response() {
    pid.config.p = P->get_value();
    pid.config.i = I->get_value();
    pid.config.d = D->get_value();
}

/// Defines a record for testing purposes, currently tests a float and int64
TestRecord::TestRecord(std::string name, double test_float, int64_t test_int64)
    : Record(std::move(name)),
      test_float(test_float),
      test_int64(test_int64),
      test_float_ptr(new Float("test_float")),
      test_int64_ptr(new Int64("test_int64")) {
    Record::set_fields({test_float_ptr, test_int64_ptr});
}

void TestRecord::fetch() {
    test_float_ptr->set_value((float)test_float);
    test_int64_ptr->set_value((int64_t)test_int64);
}

void TestRecord::response() {
    test_float = test_float_ptr->get_value();
    test_int64 = test_int64_ptr->get_value();
}

}  // namespace VDP
