
#pragma once

#include <units/length.h>
#include <units/velocity.h>

#include "rmb/motorcontrol/Conversions.h"

class AngularEncoder;

namespace rmb {
class LinearEncoder {
public:
  /**
   * Common interface for returning the linear velocity of an encoder.
   *
   * @return The velocity of the encoder in meters per second.
   */
  virtual units::meters_per_second_t getVelocity() const = 0;

  /**
   * Common interface for returning the linear position of an encoder.
   *
   * @return The position of the encoder in meters.
   */
  virtual units::meter_t getPosition() const  = 0;

    /**
   * Common interface for zeroing the linear positon an encoder so the current
   * position is set to the offset.
   *
   * @param offset the offset from the current angular position at which to 
   *               set the zero position.
   */
  virtual void zeroPosition(units::meter_t offset = 0_m) = 0;
};

/**
 * Generates a `AngularEncoder` to controller from an 
 * `LinearEncoder` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::shared_ptr<AngularEncoder> asAngular(std::shared_ptr<LinearEncoder> linearEncoder, 
                                          MotorControlConversions::ConversionUnit_t conversion);
} // namespace rmb