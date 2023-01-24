#pragma once

#include <limits>
#include <functional>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <rev/CANSparkMax.h>

#include "rmb/motorcontrol/feedback/AngularPositionFeedbackController.h"
#include "rmb/motorcontrol/feedforward/SimpleFeedforward.h"

namespace rmb {
// THERES A BUG IN GCC! AAARRGGHHHH!
namespace SparkMaxPositionControllerHelper {
  struct MotorConfig {
    int id;
    rev::CANSparkMax::MotorType motorType = rev::CANSparkMax::MotorType::kBrushless;
    bool inverted = false;
  };

  struct PIDConfig {
    double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
    units::turn_t tolerance = 0.0_rad;
    double iZone = 0.0, iMaxAccumulator = 0.0;
    double maxOutput = 1.0, minOutput = -1.0;
  };

  struct Range {
    units::radian_t maxPosition = std::numeric_limits<units::radian_t>::infinity();
    units::radian_t minPosition = -std::numeric_limits<units::radian_t>::infinity();
    bool isContinouse = false;
  };

  const rmb::SimpleFeedforward<units::radians> defaultFeedforward;

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
}

/**
 * A wrapper around the SparkMax motorcontroller that allows for the user to set
 * and get the position of the motor accurately through PID functionallity
 */
class SparkMaxPositionController : AngularPositionFeedbackController {
public:
  using MotorConfig = SparkMaxPositionControllerHelper::MotorConfig;
  using PIDConfig = SparkMaxPositionControllerHelper::PIDConfig;
  using Range = SparkMaxPositionControllerHelper::Range;
  using ProfileConfig = SparkMaxPositionControllerHelper::ProfileConfig;
  using EncoderType = SparkMaxPositionControllerHelper::EncoderType;
  using LimitSwitchConfig = SparkMaxPositionControllerHelper::LimitSwitchConfig;
  using FeedbackConfig = SparkMaxPositionControllerHelper::FeedbackConfig;

  SparkMaxPositionController(SparkMaxPositionController&&) = delete;
  SparkMaxPositionController(const SparkMaxPositionController&) = delete;
    
  SparkMaxPositionController(const MotorConfig motorConfig, const PIDConfig pidConfig = {}, 
                             const rmb::Feedforward<units::radians>& feedforward = SparkMaxPositionControllerHelper::defaultFeedforward,
                             const Range range = {}, const ProfileConfig profileConfig = {}, 
                             const FeedbackConfig feedbackConfig = {}, std::initializer_list<const MotorConfig> followers = {},
                             std::function<void(rev::CANSparkMax&)> customConfig = [](rev::CANSparkMax&){ return; });

  //--------------------
  // Controller Methods
  //--------------------

  /**
   * Setting the target position. 
   * 
   * @param position The target position in radians.
   */
  void setPosition(units::radian_t position);

  /**
   * Gets the target position.
   * 
   * @return The target position in radians.
   */
  units::radian_t getTargetPosition() const;

  /**
   * Gets the minimum position.
   * 
   * @return The minimum position in radians.
   */
  units::radian_t getMinPosition() const;

  /**
   * Gets the maximum position.
   * 
   * @return The maximum position in radians.
   */
  units::radian_t getMaxPosition() const;

  /**
   * Disables the motor.
   */
  void disable();

  /**
   * Stops the motor until `setPosition` is called again.
   */
  void stop();

  //-----------------
  // Encoder Methods
  //-----------------

  /**
   * Gets the velocity of the motor.
   *
   * @return The velocity of the motor in radians per second.
   */
  units::radians_per_second_t getVelocity() const;

  /**
   * Gets the position of the motor.
   *
   * @return The position of the motor in radians.
   */
  units::radian_t getPosition() const;

  /**
   * Zeros the positon th emotor so the current position is set to the offset.
   *
   * @param offset the offset from the current position at which to set the 
   *               zero position.
   */
  void zeroPosition(units::radian_t offset = 0_rad);

  //-----------------------------
  // Feedbakc Controller Methods
  //-----------------------------

  /**
   * Gets a controllers tolerance
   * 
   * @return tolerance in radians
   */
  units::radian_t getTolerance() const;

private:
  rev::CANSparkMax sparkMax;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;
  rev::CANSparkMax::ControlType controlType;

  rev::SparkMaxPIDController pidController;
  units::radian_t targetPosition;
  units::radian_t tolerance;

  std::unique_ptr<rev::MotorFeedbackSensor> encoder;
  EncoderType encoderType;
  double gearRatio;
};
} // namespace rmb