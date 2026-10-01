#pragma once

#include "mantle/sensor.h"
#include "mantle/subsystems/custom_encoder.h"
#include "vex.h"


namespace crust {

/**
 * Concrete VEX V5 Encoder driver implementing mantle::Encoder.
 */
class V5Encoder : public mantle::Encoder {
private:
    vex::encoder encoder;

public:
    explicit V5Encoder(vex::triport::port& port) : encoder(port) {}
    explicit V5Encoder(vex::encoder enc) : encoder(enc) {}

    double position(core::RotationUnits units) override {
        vex::rotationUnits r_u = (units == core::RotationUnits::Rotations) ? vex::rotationUnits::rev : vex::rotationUnits::deg;
        return encoder.position(r_u);
    }

    void reset() override {
        encoder.resetRotation();
    }

    vex::encoder& raw_encoder() { return encoder; }
};

/**
 * Concrete VEX V5 Smart Port Rotation Sensor driver implementing mantle::Encoder.
 */
class V5RotationSensor : public mantle::Encoder {
private:
    vex::rotation sensor;

public:
    explicit V5RotationSensor(int32_t port, bool reversed = false) : sensor(port, reversed) {}
    explicit V5RotationSensor(vex::rotation rot) : sensor(rot) {}

    double position(core::RotationUnits units) override {
        vex::rotationUnits r_u = (units == core::RotationUnits::Rotations) ? vex::rotationUnits::rev : vex::rotationUnits::deg;
        return sensor.position(r_u);
    }

    void reset() override {
        sensor.resetPosition();
    }

    vex::rotation& raw_sensor() { return sensor; }
};


/**
 * Concrete VEX V5 Inertial sensor driver implementing mantle::InertialSensor.
 */
class V5Inertial : public mantle::InertialSensor {
private:
    vex::inertial imu;

public:
    explicit V5Inertial(int32_t port) : imu(port) {}
    explicit V5Inertial(vex::inertial i) : imu(i) {}

    double heading(core::RotationUnits units) override {
        vex::rotationUnits r_u = (units == core::RotationUnits::Rotations) ? vex::rotationUnits::rev : vex::rotationUnits::deg;
        return imu.heading(r_u);
    }

    double rotation(core::RotationUnits units) override {
        vex::rotationUnits r_u = (units == core::RotationUnits::Rotations) ? vex::rotationUnits::rev : vex::rotationUnits::deg;
        return imu.rotation(r_u);
    }

    bool is_calibrating() override {
        return imu.isCalibrating();
    }

    bool is_installed() override {
        return imu.installed();
    }

    void reset_rotation() override {
        imu.resetRotation();
    }

    void calibrate() {
        imu.calibrate();
    }

    vex::inertial& raw_imu() { return imu; }
};

/**
 * Concrete VEX V5 Potentiometer driver implementing mantle::Potentiometer.
 */
class V5Potentiometer : public mantle::Potentiometer {
private:
    vex::pot sensor;

public:
    explicit V5Potentiometer(vex::triport::port& port) : sensor(port) {}
    explicit V5Potentiometer(vex::pot p) : sensor(p) {}

    double value(core::PercentUnits = core::PercentUnits::Pct) override {
        return sensor.value(vex::percentUnits::pct);
    }

    vex::pot& raw_potentiometer() { return sensor; }
};

/**
 * Concrete VEX V5 Limit switch driver implementing mantle::DigitalIn.
 */
class V5Limit : public mantle::DigitalIn {
private:
    vex::limit sensor;

public:
    explicit V5Limit(vex::triport::port& port) : sensor(port) {}
    explicit V5Limit(vex::limit l) : sensor(l) {}

    bool pressing() override {
        return sensor.pressing();
    }

    vex::limit& raw_limit() { return sensor; }
};

/**
 * Concrete VEX V5 Bumper switch driver implementing mantle::DigitalIn.
 */
class V5Bumper : public mantle::DigitalIn {
private:
    vex::bumper sensor;

public:
    explicit V5Bumper(vex::triport::port& port) : sensor(port) {}
    explicit V5Bumper(vex::bumper b) : sensor(b) {}

    bool pressing() override {
        return sensor.pressing();
    }

    vex::bumper& raw_bumper() { return sensor; }
};

/**
 * Concrete VEX V5 Custom Quad Encoder with tick scaling implementing mantle::Encoder.
 */
class V5CustomEncoder : public mantle::CustomEncoder {
private:
    V5Encoder v5_enc;

public:
    V5CustomEncoder(vex::triport::port& port, double ticks_per_rev)
        : mantle::CustomEncoder(ticks_per_rev), v5_enc(port) {
        set_base_encoder(&v5_enc);
    }

    vex::encoder& raw_encoder() { return v5_enc.raw_encoder(); }
};

} // namespace crust


