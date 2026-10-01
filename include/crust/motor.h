#pragma once

#include "mantle/motor.h"
#include "vex.h"

namespace crust {

/**
 * Concrete VEX V5 Motor driver implementing the mantle::Motor interface.
 */
class V5Motor : public mantle::Motor {
private:
    vex::motor motor;

public:
    explicit V5Motor(int32_t port, bool reversed = false, vex::gearSetting gear = vex::ratio18_1)
        : motor(port, gear, reversed) {}

    explicit V5Motor(vex::motor m) : motor(m) {}

    void spin(core::Direction dir, double val, core::VoltageUnits units) override {
        vex::directionType v_dir = (dir == core::Direction::Forward) ? vex::directionType::fwd : vex::directionType::rev;
        vex::voltageUnits v_u = (units == core::VoltageUnits::Millivolt) ? vex::voltageUnits::mV : vex::voltageUnits::volt;
        motor.spin(v_dir, val, v_u);
    }

    void spin_velocity(core::Direction dir, double val, core::VelocityUnits units) override {
        vex::directionType v_dir = (dir == core::Direction::Forward) ? vex::directionType::fwd : vex::directionType::rev;
        vex::velocityUnits v_u = (units == core::VelocityUnits::Pct) ? vex::velocityUnits::pct : vex::velocityUnits::rpm;
        motor.spin(v_dir, val, v_u);
    }

    void stop(core::BrakeMode mode) override {
        vex::brakeType b_mode = vex::brakeType::coast;
        if (mode == core::BrakeMode::Brake) b_mode = vex::brakeType::brake;
        else if (mode == core::BrakeMode::Hold) b_mode = vex::brakeType::hold;
        motor.stop(b_mode);
    }

    double position(core::RotationUnits units) override {
        vex::rotationUnits r_u = (units == core::RotationUnits::Degrees) ? vex::rotationUnits::deg : vex::rotationUnits::rev;
        return motor.position(r_u);
    }

    double velocity(core::VelocityUnits units) override {
        vex::velocityUnits v_u = (units == core::VelocityUnits::Pct) ? vex::velocityUnits::pct : vex::velocityUnits::rpm;
        return motor.velocity(v_u);
    }

    double temperature() override {
        return motor.temperature(vex::temperatureUnits::celsius);
    }

    double current() override {
        return motor.current(vex::currentUnits::amp);
    }

    double voltage() override {
        return motor.voltage(vex::voltageUnits::volt);
    }

    void reset_position() override {
        motor.resetPosition();
    }

    vex::motor& raw_motor() { return motor; }
};

} // namespace crust
