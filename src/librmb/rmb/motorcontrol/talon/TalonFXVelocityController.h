
#pragma once

#include <functional>

#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#include "rmb/motorcontrol/feedback/AngularVelocityFeedbackController.h"

namespace rmb {

namespace TalonFXVelocityControllerHelper {

  struct MotorConfig {
    int id;
    bool inverted;
  };

  struct PIDConfig {
    double p = 0.0, i = 0.0, d = 0.0, f = 0.0;
    units::radians_per_second_t tolerance = 0.0_rad_per_s;
    double iZone = 0.0, iMaxAccumulator = 0.0;
    double maxOutput = 1.0, minOutput = -1.0;
  };

  enum LimitSwitchConfig { Disabled, NormalyOpen, NormalyClosed };

  struct FeedbackConfig {
    double gearRatio = 1.0;
    LimitSwitchConfig forwardSwitch = Disabled, reverseSwitch = Disabled;
  };
}

class TalonFXVelocityController : public AngularVelocityFeedbackController {
public:

  using EncoderTick = units::unit<std::ratio<4096, 1>, units::turns>;
  using EncoderTick_t = units::unit_t<EncoderTick>;
  using EncoderVelocity = units::compound_unit<EncoderTick, units::inverse<units::decisecond>>;
  using EncoderVelocity_t  = units::unit_t<EncoderVelocity>;

  using MotorConfig = TalonFXVelocityControllerHelper::MotorConfig;
  using PIDConfig = TalonFXVelocityControllerHelper::PIDConfig;
  using LimitSwitchConfig = TalonFXVelocityControllerHelper::LimitSwitchConfig;
  using FeedbackConfig = TalonFXVelocityControllerHelper::FeedbackConfig;

  TalonFXVelocityController(const MotorConfig motorConfig = {}, const PIDConfig pidConfig = {}, 
                            const FeedbackConfig feedbackConfig = {}, std::initializer_list<const MotorConfig> followers = {},
                            std::function<void(ctre::phoenix::motorcontrol::can::WPI_TalonFX&)> customConfig = [](ctre::phoenix::motorcontrol::can::WPI_TalonFX&) { return 0; });



  //--------------------------------------------------
  // Methods Inherited from AngularVelocityController
  //--------------------------------------------------

  /**
   * Sets the target angular velocity.
   * 
   * @param velocity The target angular velocity in radians per second.
   */
  void setVelocity(units::radians_per_second_t velocity) override;

  /**
   * Gets the target angular velocity.
   * 
   * @return The target angular velocity in radians per second.
   */
  units::radians_per_second_t getTargetVelocity() const override;

  /**
   * Gets the maximum angular velocity.
   * 
   * @return The maximum angular velocity in radians per second.
   */
  units::radians_per_second_t getMaxVelocity() const override;

  /**
   * Common interface for setting a mechanism's raw power output.
   */
  void setPower(double power) override;

  /**
   * Common interface for disabling a mechanism.
   */
  void disable() override;

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  void stop() override;

  //---------------------------------------
  // Methods Inherited from AngularEncoder
  //---------------------------------------

  /**
   * Gets the angular velocity of the motor.
   *
   * @return The velocity of the motor in radians per second.
   */
  units::radians_per_second_t getVelocity() const override;

  /**
   * Gets the angular position of an motor.
   *
   * @return The position of the motor in radians.
   */
  units::radian_t getPosition() const override;

  /**
   * Zeros the angular positon the motor so the current position is set to 
   * the offset.
   *
   * @param offset the offset from the current angular position at which to 
   *               set the zero position.
   */
  void zeroPosition(units::radian_t offset = 0_rad) override;

  //----------------------------------------------------------
  // Methods Inherited from AngularvelocityFeedbackController
  //----------------------------------------------------------

  /**
   * Gets the motor's tolerance.
   * 
   * @return the motor's tolerance in radians per second.
   */
  units::radians_per_second_t getTolerance() const override;

private:
  ctre::phoenix::motorcontrol::can::WPI_TalonFX talonFX;

  double gearRatio;
};
} // namespace rmb