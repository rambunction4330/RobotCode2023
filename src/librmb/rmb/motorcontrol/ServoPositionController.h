
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

#include <frc/Servo.h>

#include "rmb/motorcontrol/AngularPositionController.h"

namespace rmb {

/**
 * Custom servo obhect the conforms to the `AngularPositionController` 
 * interface.
 */
class ServoPositionController : public AngularPositionController {
public:

  ServoPositionController(const ServoPositionController&) = delete;
  ServoPositionController(ServoPositionController&&) = default;

  ServoPositionController(int channel) : servo(channel) {} 

  /**
   * Common interface for setting the target linear position. 
   * 
   * @param position The target linear position in meters.
   */
  virtual void setPosition(units::radian_t position) {
    units::radian_t tagrePosition = std::clamp(position * inversion, minPosition, maxPosition);
    return servo.SetAngle(units::degree_t(tagrePosition).to<double>());
  }

  /**
   * Common interface for getting the <b>target</b> linear position.
   * 
   * @return The <b>target</b> linear position in meters.
   */
  virtual units::radian_t getTargetPosition() const {
    return units::degree_t(servo.GetAngle() * inversion);
  }

  /**
   * Sets the minimum angular position.
   * 
   * @param min The minimum angular position in radians.
   */
  virtual void setMinPosition(units::radian_t min) {
    minPosition = min;
  }

  /**
   * Gets the minimum angular position.
   * 
   * @return The minimum angular position in radians.
   */
  virtual units::radian_t getMinPosition() const {
    return minPosition;
  }

  /**
   * Sets the maximum angular position.
   * 
   * @param max  The maximum angular position in radians.
   */
  virtual void setMaxPosition(units::radian_t max) {
    maxPosition = max;
  }

  /**
   * Gets the maximum angular position.
   * 
   * @return The maximum angular position in radians.
   */
  virtual units::radian_t getMaxPosition() const {
    return maxPosition;
  }

  /**
   * Common interface for inverting direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  virtual void setInverted(bool isInverted) {
    inversion = isInverted ? -1 : 1;
  }

    /**
   * Common interface for returning the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  virtual bool getInverted() const {
    return inversion == 1;
  }

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() {
    servo.SetDisabled();
  }

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() {
    servo.SetDisabled();
  }

private:
  frc::Servo servo;
  int inversion = 1;
  units::radian_t minPosition = 0_deg;
  units::radian_t maxPosition = 180_deg;
};
} // namespace rmb
