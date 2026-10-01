#pragma once

#include "core/utils/units.h"
#include <memory>

namespace mantle {

/**
 * Hardware Abstraction Layer interface for rotation encoders.
 */
class Encoder {
public:
    virtual ~Encoder() = default;

    virtual double position(core::RotationUnits units = core::RotationUnits::Degrees) = 0;
    virtual void reset() = 0;
};

/**
 * Hardware Abstraction Layer interface for inertial sensors (IMU).
 */
class InertialSensor {
public:
    virtual ~InertialSensor() = default;

    virtual double heading(core::RotationUnits units = core::RotationUnits::Degrees) = 0;
    virtual double rotation(core::RotationUnits units = core::RotationUnits::Degrees) = 0;
    virtual bool is_calibrating() = 0;
    virtual bool is_installed() { return true; }
    virtual void reset_rotation() = 0;
};

/**
 * Hardware Abstraction Layer interface for digital binary inputs (limit switches, bumpers).
 */
class DigitalIn {
public:
    virtual ~DigitalIn() = default;
    virtual bool pressing() = 0;
    virtual bool value() { return pressing(); }
};

/**
 * Hardware Abstraction Layer interface for analog potentiometers.
 */
class Potentiometer {
public:
    virtual ~Potentiometer() = default;
    virtual double value(core::PercentUnits units = core::PercentUnits::Pct) = 0;
};

/**
 * In-memory mock digital input for testing.
 */
class MockDigitalIn : public DigitalIn {
public:
    bool state = false;
    bool pressing() override { return state; }
};


/**
 * In-memory mock encoder for testing.
 */
class MockEncoder : public Encoder {
public:
    double stored_degrees = 0.0;

    double position(core::RotationUnits units) override {
        if (units == core::RotationUnits::Rotations) return stored_degrees / 360.0;
        return stored_degrees;
    }

    void reset() override {
        stored_degrees = 0.0;
    }
};

/**
 * In-memory mock inertial sensor for testing.
 */
class MockInertial : public InertialSensor {
public:
    double stored_degrees = 0.0;
    bool calibrating = false;

    double heading(core::RotationUnits units) override {
        double deg = std::fmod(stored_degrees, 360.0);
        if (deg < 0.0) deg += 360.0;
        if (units == core::RotationUnits::Rotations) return deg / 360.0;
        return deg;
    }

    double rotation(core::RotationUnits units) override {
        if (units == core::RotationUnits::Rotations) return stored_degrees / 360.0;
        return stored_degrees;
    }

    bool is_calibrating() override { return calibrating; }
    void reset_rotation() override { stored_degrees = 0.0; }
};

} // namespace mantle
