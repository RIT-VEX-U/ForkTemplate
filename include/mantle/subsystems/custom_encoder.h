#pragma once

#include "mantle/sensor.h"
#include "core/utils/units.h"
#include <memory>

namespace mantle {

/**
 * A wrapper class for encoders that allows the use of 3rd party
 * encoders with different tick-per-revolution values.
 * ZERO VEX dependencies.
 */
class CustomEncoder : public Encoder {
public:
  /**
   * Construct a custom scaled encoder
   * @param ticks_per_rev the number of ticks the encoder will report for one revolution
   * @param base_encoder optional underlying encoder to scale readings from
   */
  explicit CustomEncoder(double ticks_per_rev = 360.0, Encoder *base_encoder = nullptr);

  void set_base_encoder(Encoder *enc);

  /**
   * sets the stored rotation of the encoder.
   */
  void setRotation(double val, core::RotationUnits units = core::RotationUnits::Degrees);

  /**
   * sets the stored position of the encoder.
   */
  void setPosition(double val, core::RotationUnits units = core::RotationUnits::Degrees);

  /**
   * get the rotation that the encoder is at
   */
  double rotation(core::RotationUnits units = core::RotationUnits::Degrees);

  /**
   * get the position that the encoder is at
   */
  double position(core::RotationUnits units = core::RotationUnits::Degrees) override;

  /**
   * get the velocity that the encoder is moving at
   */
  double velocity(core::VelocityUnits units = core::VelocityUnits::Rpm);

  void reset() override;

private:
  Encoder *base_encoder = nullptr;
  double tick_scalar = 1.0;
  double offset_degrees = 0.0;
};

} // namespace mantle

using CustomEncoder = mantle::CustomEncoder;