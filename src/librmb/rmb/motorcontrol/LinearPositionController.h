
#pragma once

#include <memory>

#include <units/length.h>

#include "rmb/motorcontrol/Conversions.h"


namespace rmb {

class AngularPositionController;

/**
 * Interface for controlling a mechanism's linear position used by wrappers of 
 * device specific APIs.
 */
class LinearPositionController {
public:

/**
   * Common interface for setting the target linear position. 
   * 
   * @param position The target linear position in meters.
   */
  virtual void setPosition(units::meter_t position) = 0;

  /**
   * Common interface for getting the <b>target</b> linear position.
   * 
   * @return The target linear position in meters.
   */
  virtual units::meter_t getTargetPosition() const = 0;

  /**
   * Common interface for getting the minimum linear position.
   * 
   * @return The minimum linear position in meters.
   */
  virtual units::meter_t getMinPosition() const = 0;

  /**
   * Common interface for getting the maximum linear position.
   * 
   * @return The maximum linear position in meters.
   */
  virtual units::meter_t getMaxPosition() const = 0;

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
 * Generates a `AngularPositionController` to controller from an 
 * `LinearPositionController` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::shared_ptr<AngularPositionController> asAngular(std::shared_ptr<LinearPositionController> linearController, 
                                                     MotorControlConversions::ConversionUnit_t conversion);

} // namespace rmb
