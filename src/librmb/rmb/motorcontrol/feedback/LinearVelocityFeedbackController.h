
#pragma once

#include <units/math.h>

#include "rmb/motorcontrol/Conversions.h"
#include "rmb/motorcontrol/feedback/LinearEncoder.h"
#include "rmb/motorcontrol/LinearVelocityController.h"

namespace rmb {

class AngularVelocityFeedbackController;

class LinearVelocityFeedbackController : 
public LinearVelocityController, public LinearEncoder {
public:

  /**
   * Common interface for getting a controllers tolerance
   */
  virtual units::meters_per_second_t getTolerance() const = 0;

  /**
   * Common interface for getting the error between the velocities controllers
   * target velocity and the actual velocity measured by the encoder.
   * 
   * @return position error in radians per second.
   */
  virtual units::meters_per_second_t getError() const {
    return getVelocity() - getTargetVelocity();
  }

  /**
   * Common interface for getting whether the mechanism has achived it's
   * target velocity. 
   * 
   * @return true is the controller has achived the target velocity.
   */
  virtual bool atTarget() const {
    return units::math::abs(getError()) < getTolerance();
  }
};

/**
 * Generates a `AngularAsLinearEncoder` to measure the same mechanism as this
 * object, but with linear instead of angular units via a linear
 * conversion factor. Changes to one controller will effect the other since 
 * they measure the same physical mechanism.
 * 
 * @param conversion conversion from linear to angular units.
 */
std::shared_ptr<AngularVelocityFeedbackController> asAngular(std::shared_ptr<LinearVelocityFeedbackController> linearController,
                                                             MotorControlConversions::ConversionUnit_t conversion);

} // namespace rmb
