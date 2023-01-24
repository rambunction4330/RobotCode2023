
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "rmb/motorcontrol/Conversions.h"

namespace rmb {
  
class LinearEncoder;

class AngularEncoder {
public:

  /**
   * Common interface for returning the angular velocity of an encoder.
   *
   * @return The velocity of the encoder in radians per second.
   */
  virtual units::radians_per_second_t getVelocity() const = 0;

  /**
   * Common interface for returning the angular position of an encoder.
   *
   * @return The position of the encoder in radians.
   */
  virtual units::radian_t getPosition() const = 0;

    /**
   * Common interface for zeroing the anguklar positon an encoder so the current
   * position is set to the offset.
   *
   * @param offset the offset from the current angular position at which to 
   *               set the zero position.
   */
  virtual void zeroPosition(units::radian_t offset = 0_rad) = 0;
};

/**
 * Generates a `LinearEncoder` to controller from an 
 * `AngularEncoder` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::shared_ptr<LinearEncoder> asLinear(std::shared_ptr<AngularEncoder> angularEncoder, 
                                        MotorControlConversions::ConversionUnit_t conversion);
} // namespace rmb