#pragma once

#include "core/utils/units.h"
#include <cmath>
#include <memory>
#include <vector>

namespace mantle {

/**
 * Hardware Abstraction Layer interface for smart motors.
 */
class Motor {
public:
    virtual ~Motor() = default;

    virtual void spin(core::Direction dir, double val, core::VoltageUnits units) = 0;
    virtual void spin_velocity(core::Direction dir, double val, core::VelocityUnits units) = 0;
    virtual void stop(core::BrakeMode mode = core::BrakeMode::Coast) = 0;
    virtual double position(core::RotationUnits units = core::RotationUnits::Rotations) = 0;
    virtual double velocity(core::VelocityUnits units = core::VelocityUnits::Rpm) = 0;
    virtual double temperature() = 0;
    virtual double current() = 0;
    virtual double voltage() = 0;
    virtual void reset_position() = 0;
};

/**
 * Group of synchronized motors acting as a single composite actuator.
 */
class MotorGroup {
private:
    std::vector<std::shared_ptr<Motor>> motors;

public:
    MotorGroup() = default;
    explicit MotorGroup(std::vector<std::shared_ptr<Motor>> m) : motors(std::move(m)) {}

    void add_motor(std::shared_ptr<Motor> motor) {
        motors.push_back(motor);
    }

    void spin(core::Direction dir, double val, core::VoltageUnits units) {
        for (auto& m : motors) {
            if (m) m->spin(dir, val, units);
        }
    }

    void spin_velocity(core::Direction dir, double val, core::VelocityUnits units) {
        for (auto& m : motors) {
            if (m) m->spin_velocity(dir, val, units);
        }
    }

    void stop(core::BrakeMode mode = core::BrakeMode::Coast) {
        for (auto& m : motors) {
            if (m) m->stop(mode);
        }
    }

    double position(core::RotationUnits units = core::RotationUnits::Rotations) {
        if (motors.empty()) return 0.0;
        double sum = 0.0;
        for (auto& m : motors) {
            if (m) sum += m->position(units);
        }
        return sum / motors.size();
    }

    double velocity(core::VelocityUnits units = core::VelocityUnits::Rpm) {
        if (motors.empty()) return 0.0;
        double sum = 0.0;
        for (auto& m : motors) {
            if (m) sum += m->velocity(units);
        }
        return sum / motors.size();
    }

    double temperature() {
        double max_t = 0.0;
        for (auto& m : motors) {
            if (m) {
                double t = m->temperature();
                if (t > max_t) max_t = t;
            }
        }
        return max_t;
    }

    double current() {
        double sum = 0.0;
        for (auto& m : motors) {
            if (m) sum += m->current();
        }
        return sum;
    }

    double voltage() {
        if (motors.empty()) return 0.0;
        double sum = 0.0;
        for (auto& m : motors) {
            if (m) sum += m->voltage();
        }
        return sum / motors.size();
    }

    void reset_position() {
        for (auto& m : motors) {
            if (m) m->reset_position();
        }
    }

    size_t size() const { return motors.size(); }
    const std::vector<std::shared_ptr<Motor>>& get_motors() const { return motors; }
};

/**
 * Mock motor implementation for testing without physical V5 hardware.
 */
class MockMotor : public Motor {
public:
    double current_voltage = 0.0;
    double current_pos = 0.0;
    double current_vel = 0.0;
    double current_temp = 25.0;
    core::BrakeMode brake_mode = core::BrakeMode::Coast;

    void spin(core::Direction dir, double val, core::VoltageUnits units) override {
        double sign = (dir == core::Direction::Forward) ? 1.0 : -1.0;
        double v = (units == core::VoltageUnits::Millivolt) ? val / 1000.0 : val;
        current_voltage = sign * v;
    }

    void spin_velocity(core::Direction dir, double val, core::VelocityUnits units) override {
        double sign = (dir == core::Direction::Forward) ? 1.0 : -1.0;
        current_vel = sign * val;
    }

    void stop(core::BrakeMode mode) override {
        brake_mode = mode;
        current_voltage = 0.0;
        current_vel = 0.0;
    }

    double position(core::RotationUnits units) override {
        if (units == core::RotationUnits::Degrees) return current_pos * 360.0;
        return current_pos;
    }

    double velocity(core::VelocityUnits) override {
        return current_vel;
    }

    double temperature() override { return current_temp; }
    double current() override { return std::abs(current_voltage) * 0.2; }
    double voltage() override { return current_voltage; }
    void reset_position() override { current_pos = 0.0; }
};

} // namespace mantle
