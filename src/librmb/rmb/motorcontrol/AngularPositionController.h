
#pragma once

#include <memory>

#include <units/angle.h>

#include "rmb/motorcontrol/Conversions.h"

namespace rmb {

class LinearPositionController;

/**
 * Interface for controlling a mechanism's angular position used by wrappers of 
 * device specific APIs.
 */
class AngularPositionController {
public:

  /**
   * Common interface for setting the target angular position. 
   * 
   * @param position The target angular position in radians.
   */
  virtual void setPosition(units::radian_t position) = 0;

  /**
   * Common interface for getting the <b>target</b> angular position.
   * 
   * @return The target angular position in radians.
   */
  virtual units::radian_t getTargetPosition() const = 0;

  /**
   * Common interface for getting the minimum angular position.
   * 
   * @return The minimum angular position in radians.
   */
  virtual units::radian_t getMinPosition() const = 0;

  /**
   * Common interface for getting the maximum angular position.
   * 
   * @return The maximum angular position in radians.
   */
  virtual units::radian_t getMaxPosition() const = 0;

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
 * Generates a `LinearPositionController` to controller from an 
 * `AngularPositionController` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::shared_ptr<LinearPositionController> asLinear(std::shared_ptr<AngularPositionController> angularController, 
                                                   MotorControlConversions::ConversionUnit_t conversion);

} // namespace rmb
