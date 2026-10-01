#include "mantle/subsystems/custom_encoder.h"

namespace mantle {

CustomEncoder::CustomEncoder(double ticks_per_rev, Encoder *base_encoder)
    : base_encoder(base_encoder), offset_degrees(0.0) {
    // bc it's a quadrature encoder, ticks per rev has to be multiplied by 4
    if (ticks_per_rev > 0) {
        tick_scalar = 360.0 / (ticks_per_rev * 4.0);
    } else {
        tick_scalar = 1.0;
    }
}

void CustomEncoder::set_base_encoder(Encoder *enc) {
    base_encoder = enc;
}

void CustomEncoder::setRotation(double val, core::RotationUnits units) {
    double deg = (units == core::RotationUnits::Rotations) ? val * 360.0 : val;
    offset_degrees = deg;
    if (base_encoder) {
        base_encoder->reset();
    }
}

void CustomEncoder::setPosition(double val, core::RotationUnits units) {
    setRotation(val, units);
}

double CustomEncoder::rotation(core::RotationUnits units) {
    return position(units);
}

double CustomEncoder::position(core::RotationUnits units) {
    double raw = base_encoder ? base_encoder->position(core::RotationUnits::Degrees) : 0.0;
    double deg = (raw * tick_scalar) + offset_degrees;
    if (units == core::RotationUnits::Rotations) {
        return deg / 360.0;
    }
    return deg;
}

double CustomEncoder::velocity(core::VelocityUnits) {
    return 0.0;
}

void CustomEncoder::reset() {
    offset_degrees = 0.0;
    if (base_encoder) {
        base_encoder->reset();
    }
}

} // namespace mantle
