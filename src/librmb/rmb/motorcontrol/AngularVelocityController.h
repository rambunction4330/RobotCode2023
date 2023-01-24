
#pragma once

#include <memory>

#include <units/angular_velocity.h>

#include "rmb/motorcontrol/Conversions.h"

namespace rmb {

class LinearVelocityController;

/**
 * Interface for controlling a mechanism's angular velocity used by wrappers of 
 * device specific APIs.
 */
class AngularVelocityController {
public:

  /**
   * Common interface for setting the target angular velocity.
   * 
   * @param velocity The target angular velocity in radians per second.
   */
  virtual void setVelocity(units::radians_per_second_t velocity) = 0;

  /**
   * Common interface for getting the <b>target</b> angular velocity.
   * 
   * @return The <b>target</b> angular velocity in radians per second.
   */
  virtual units::radians_per_second_t getTargetVelocity() const = 0;

  /**
   * Common interface for getting the maximum angular velocity.
   * 
   * @return The maximum angular velocity in radianss per second.
   */
  virtual units::radians_per_second_t getMaxVelocity() const = 0;

  /**
   * Common interface for setting a mechanism's raw power output.
   */
  virtual void setPower(double power) = 0;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() = 0;

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() = 0;
  
};

/**
 * Generates a `LinearVelocityController` to controller from an 
 * `AngularVelocityController` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::shared_ptr<LinearVelocityController> asLinear(std::shared_ptr<AngularVelocityController> angularController,
                                                   MotorControlConversions::ConversionUnit_t conversion);

} // namespace rmb
