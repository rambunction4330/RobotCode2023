#pragma once

#include "FalconPositionController.h"
#include "rmb/motorcontrol/feedback/AngularVelocityFeedbackController.h"
#include "units/angular_velocity.h"

namespace rmb {

namespace FalconVelocityControllerHelper {
struct PIDConfig {
  double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
  units::turns_per_second_t tolerance = 0.0_tps;
  double iZone = 0.0, iMaxAccumulator = 0.0;
  double maxOutput = 1.0, minOutput = -1.0;
};

struct ProfileConfig {
  units::radians_per_second_t maxVelocity = 0.0_rad_per_s,
                              minVelocity = 0.0_rad_per_s;
  units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
};
} // namespace FalconVelocityControllerHelper

class FalconVelocityController : public AngularVelocityFeedbackController {
public:
  typedef units::unit<std::ratio<2048, 1>, units::turns> EncoderTick;
  typedef units::unit_t<EncoderTick> EncoderTick_t;

  typedef units::compound_unit<EncoderTick, units::inverse<units::deciseconds>>
      RawVelocityUnit;
  typedef units::unit_t<RawVelocityUnit> RawVelocityUnit_t;

  typedef units::unit<std::ratio<1, 1>, EncoderTick> RawPositionUnit;
  typedef units::unit_t<RawPositionUnit> RawPositionUnit_t;

  FalconVelocityController(
      FalconPositionControllerHelper::MotorConfig config,
      FalconVelocityControllerHelper::PIDConfig pidConfig,
      FalconVelocityControllerHelper::ProfileConfig profileConfig,
      FalconPositionControllerHelper::FeedbackConfig feedbackConfig);

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
   * the offset. In the case of an absolute encoder this sets the zero offset
   * with no regard to the current position.
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
  mutable ctre::phoenix::motorcontrol::can::WPI_TalonFX motorcontroller;

  float gearRatio = 0.0;

  units::radians_per_second_t tolerance = 0.0_tps;

  FalconVelocityControllerHelper::ProfileConfig profileConfig;
};

} // namespace rmb
