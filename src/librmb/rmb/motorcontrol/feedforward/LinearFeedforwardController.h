
#pragma once

#include <algorithm>
#include <limits>

#include <frc/motorcontrol/MotorController.h>

#include "rmb/motorcontrol/LinearVelocityController.h"
#include "rmb/motorcontrol/feedforward/Feedforward.h"

namespace rmb {

/**
 * Interface for setting a motor controllers linear velocity using a feedfoward. 
 * <b>Beware<\b> that since there is no feedbakc device several functions will 
 * behave incorrectly. Additionaly, an `update` method may need to be added for
 * proper voltage compenstation.
 */
class LinearFeedforwardController : public LinearVelocityController{
public:

  LinearFeedforwardController(const LinearFeedforwardController&) = delete;
  LinearFeedforwardController(LinearFeedforwardController&&) = default;

  LinearFeedforwardController(std::unique_ptr<frc::MotorController>&& motorController, 
                              std::unique_ptr<rmb::Feedforward<units::meters>>&& feedforward) : 
                              motorController(std::move(motorController)), feedforward(std::move(feedforward)) {}

  /**
   * Sets the target velocity.
   * 
   * @param velocity The target linear velocity in meters per second.
   */
  void setVelocity(units::meters_per_second_t velocity) {
    targetVelocity = std::clamp(velocity, -maxVelocity, maxVelocity);
    motorController->SetVoltage(feedforward->calculate(targetVelocity));
  }

  /**
   * Gets the <b>target</b> velocity.
   * 
   * @return The <b>target</b> velocity in meters per second.
   */
  units::meters_per_second_t getTargetVelocity() const { return targetVelocity; }

  /**
   * Sets the maximum angular velocity.
   * 
   * @param max The maximum angular velocity in radians per second.
   */
  void setMaxVelocity(units::meters_per_second_t max) {
    maxVelocity = max;
  }

  /**
   * Gets the maximum angular velocity.
   * 
   * @return The maximum angular velocity in radianss per second.
   */
  units::meters_per_second_t getMaxVelocity() const {
    return maxVelocity;
  }

  /**
   *Inverterts the direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  void setInverted(bool isInverted) { motorController->SetInverted(isInverted); }

  /**
   * Returns the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  bool getInverted() const { motorController->GetInverted(); }

  /**
   * Disabls a mechanism.
   */
  void disable() { 
    targetVelocity = 0.0_mps;
    motorController->Disable(); 
  }

  /**
   * Stops the mechanism until `setVelocity` is called again.
   */
  void stop() { 
    targetVelocity = 0.0_mps;
    motorController->StopMotor(); 
  }

private:
  std::unique_ptr<frc::MotorController> motorController;
  std::unique_ptr<rmb::Feedforward<units::meters>> feedforward;
  units::meters_per_second_t targetVelocity = 0.0_mps;
  units::meters_per_second_t maxVelocity = units::radians_per_second_t(std::numeric_limits<double>::infinity());
};
} // namespace rmb
