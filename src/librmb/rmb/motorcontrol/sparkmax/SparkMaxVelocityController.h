#pragma once

#include <initializer_list>
#include <functional>

#include <rev/CANSparkMax.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include "rmb/motorcontrol/feedback/AngularVelocityFeedbackController.h"
#include "rmb/motorcontrol/feedforward/Feedforward.h"
#include "rmb/motorcontrol/feedforward/SimpleFeedforward.h"

namespace rmb {

// THERES A BUG IN GCC! AAARRGGHHHH!
namespace SparkMaxVelocityControllerHelper {
  struct MotorConfig {
    int id;
    rev::CANSparkMax::MotorType motorType = rev::CANSparkMax::MotorType::kBrushless;
    bool inverted = false;
  };

  struct PIDConfig {
    double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
    units::radians_per_second_t tolerance = 0.0_rad_per_s;
    double iZone = 0.0, iMaxAccumulator = 0.0;
    double maxOutput = 1.0, minOutput = -1.0;
  };

  struct ProfileConfig {
    bool useSmartMotion = false;
    units::radians_per_second_t maxVelocity = 0.0_rad_per_s, minVelocity = 0.0_rad_per_s;
    units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
    rev::SparkMaxPIDController::AccelStrategy accelStrategy = rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal;
  };

  enum EncoderType { HallSensor, Quadrature, Alternate, Absolute };
  enum LimitSwitchConfig { Disabled, NormalyOpen, NormalyClosed };

  struct FeedbackConfig {
    double gearRatio = 1.0;
    EncoderType encoderType = HallSensor;
    int countPerRev = 42;
    LimitSwitchConfig forwardSwitch = Disabled, reverseSwitch = Disabled;
  };

  const rmb::SimpleFeedforward<units::radians> defaultFeedforward;
}

/**
 * A wrapper around the SparkMax motorcontroller so that it can complies with 
 * the `AngularVelocityFeedbackController` interface.
 */
class SparkMaxVelocityController : public AngularVelocityFeedbackController {
public:
  using MotorConfig = SparkMaxVelocityControllerHelper::MotorConfig;
  using PIDConfig = SparkMaxVelocityControllerHelper::PIDConfig;
  using ProfileConfig = SparkMaxVelocityControllerHelper::ProfileConfig;
  using EncoderType = SparkMaxVelocityControllerHelper::EncoderType;
  using LimitSwitchConfig = SparkMaxVelocityControllerHelper::LimitSwitchConfig;
  using FeedbackConfig = SparkMaxVelocityControllerHelper::FeedbackConfig;
  
  SparkMaxVelocityController(SparkMaxVelocityController&&) = delete;
  SparkMaxVelocityController(const SparkMaxVelocityController&) = delete;
    
  SparkMaxVelocityController(const MotorConfig motorConfig, const PIDConfig pidConfig = {}, 
                             const ProfileConfig profileConfig = {}, const FeedbackConfig feedbackConfig = {}, 
                             std::initializer_list<const MotorConfig> followers = {},
                             std::function<void(rev::CANSparkMax&)> customConfig = [](rev::CANSparkMax&){ return; });

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
  virtual void setPower(double power) override;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() override;

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() override;

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
  virtual units::radians_per_second_t getTolerance() const override;

private:
  rev::CANSparkMax sparkMax;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;
  rev::CANSparkMax::ControlType controlType;

  rev::SparkMaxPIDController pidController;
  units::radians_per_second_t targetVelocity = 0.0_rpm;
  units::radians_per_second_t tolerance;
  units::radians_per_second_t maxVelcoity;

  std::unique_ptr<rev::MotorFeedbackSensor> encoder;
  EncoderType encoderType;
  double gearRatio;
};
} // namespace rmb
