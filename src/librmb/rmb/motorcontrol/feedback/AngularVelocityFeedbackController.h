
#pragma once

#include <units/math.h>

#include "rmb/motorcontrol/Conversions.h"
#include "rmb/motorcontrol/feedback/AngularEncoder.h"
#include "rmb/motorcontrol/AngularVelocityController.h"

namespace rmb {

class LinearVelocityFeedbackController;

class AngularVelocityFeedbackController : 
public AngularVelocityController, public AngularEncoder {
public:

  /**
   * Common interface for getting a controllers tolerance
   */
  virtual units::radians_per_second_t getTolerance() const = 0;

  /**
   * Common interface for getting the error between the velocities controllers
   * target velocity and the actual velocity measured by the encoder.
   * 
   * @return position error in radians per second.
   */
  virtual units::radians_per_second_t getError() const {
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
 * Generates a `LinearVelocityFeedbackController` to controller from an 
 * `AngularVelocityFeedbackController` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::shared_ptr<LinearVelocityFeedbackController> asLinear(std::shared_ptr<AngularVelocityFeedbackController> angularController, 
                                                           MotorControlConversions::ConversionUnit_t conversion);

} // namespace rmb
