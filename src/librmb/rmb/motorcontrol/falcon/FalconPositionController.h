#pragma once

#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/feedback/AngularPositionFeedbackController.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "units/base.h"
#include "units/time.h"

namespace rmb {
namespace FalconPositionControllerHelper {
struct MotorConfig {
  int id;
  bool inverted = false;
};

struct PIDConfig {
  double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
  units::turn_t tolerance = 0.0_rad;
  double iZone = 0.0, iMaxAccumulator = 0.0;
  double maxOutput = 1.0, minOutput = -1.0;
};

struct Range {
  units::radian_t minPosition =
      -std::numeric_limits<units::radian_t>::infinity();
  units::radian_t maxPosition =
      std::numeric_limits<units::radian_t>::infinity();
  bool isContinouse = false;
};

struct ProfileConfig {
  bool useSmartMotion = false;
  units::radians_per_second_t maxVelocity = 0.0_rad_per_s,
                              minVelocity = 0.0_rad_per_s;
  units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
  /*rev::SparkMaxPIDController::AccelStrategy accelStrategy =
      rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal;*/
};

enum LimitSwitchConfig { Disabled, NormalyOpen, NormalyClosed };

struct FeedbackConfig {
  double gearRatio = 1.0;
  LimitSwitchConfig forwardSwitch = Disabled, reverseSwitch = Disabled;
};
} // namespace FalconPositionControllerHelper
class FalconPositionController : public AngularPositionFeedbackController {
public:
  typedef units::unit<std::ratio<2048, 1>, units::turns> EncoderTick;
  typedef units::unit_t<EncoderTick> EncoderTick_t;

  typedef units::compound_unit<EncoderTick, units::inverse<units::deciseconds>> RawVelocityUnit;
  typedef units::unit_t<RawVelocityUnit> RawVelocityUnit_t;

  typedef units::unit<std::ratio<1, 1>, EncoderTick> RawPositionUnit;
  typedef units::unit_t<RawPositionUnit> RawPositionUnit_t;

  FalconPositionController(FalconPositionControllerHelper::MotorConfig config,
                           FalconPositionControllerHelper::PIDConfig pidConfig,
                           FalconPositionControllerHelper::Range range,
                           FalconPositionControllerHelper::FeedbackConfig feedbackConfig);

  void setPosition(units::radian_t position) override;

  units::radian_t getTargetPosition() const override;

  units::radian_t getMinPosition() const override;

  units::radian_t getMaxPosition() const override;

  void disable() override;

  void stop() override;

  units::radians_per_second_t getVelocity() const override;

  units::radian_t getPosition() const override;

  void zeroPosition(units::radian_t offset = 0_rad) override;

  units::radian_t getTolerance() const override;

private:
  mutable ctre::phoenix::motorcontrol::can::WPI_TalonFX motorcontroller;

  FalconPositionControllerHelper::Range range;

  float gearRatio = 0.0;

  units::radian_t offset = 0_rad;

  units::radian_t tolerance = 0.0_rad;
};
} // namespace rmb
