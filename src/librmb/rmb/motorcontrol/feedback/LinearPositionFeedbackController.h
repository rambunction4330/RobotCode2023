
#pragma once

#include <units/math.h>

#include "rmb/motorcontrol/Conversions.h"
#include "rmb/motorcontrol/feedback/LinearEncoder.h"
#include "rmb/motorcontrol/LinearPositionController.h"

namespace rmb {

class AngularPositionFeedbackController;

class LinearPositionFeedbackController : 
public LinearPositionController, public LinearEncoder {
public:

  /**
   * Common interface for getting a controllers tolerance
   */
  virtual units::meter_t getTolerance() const = 0;

  /**
   * Common interface for getting the error between the velocities controllers
   * target velocity and the actual velocity measured by the encoder.
   * 
   * @return position error in radians per second.
   */
  virtual units::meter_t getError() const {
    return getPosition() - getTargetPosition();
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
 * Generates a `AngularEncoder` to controller from an 
 * `LinearEncoder` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::shared_ptr<AngularPositionFeedbackController> asAngular(std::shared_ptr<LinearPositionFeedbackController> linearController,
                                                             MotorControlConversions::ConversionUnit_t conversion);
} // namespace rmb
