#include "FalconVelocityController.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "units/angular_velocity.h"

namespace rmb {

FalconVelocityController::FalconVelocityController(
    FalconPositionControllerHelper::MotorConfig config,
    FalconVelocityControllerHelper::PIDConfig pidConfig,
    FalconVelocityControllerHelper::ProfileConfig profileConfig,
    FalconPositionControllerHelper::FeedbackConfig feedbackConfig)
    : motorcontroller(config.id) {

  motorcontroller.SetInverted(config.inverted);

  motorcontroller.Config_kD(0, pidConfig.d);
  motorcontroller.Config_kI(0, pidConfig.i);
  motorcontroller.Config_kP(0, pidConfig.p);
  motorcontroller.Config_kF(0, pidConfig.ff);
  motorcontroller.ConfigAllowableClosedloopError(
      0, RawVelocityUnit_t(pidConfig.tolerance)());

  motorcontroller.Config_IntegralZone(0, pidConfig.iZone);
  motorcontroller.ConfigMaxIntegralAccumulator(0, pidConfig.iMaxAccumulator);
  motorcontroller.ConfigPeakOutputForward(0, pidConfig.maxOutput);
  motorcontroller.ConfigPeakOutputReverse(0, pidConfig.minOutput);

  motorcontroller.ConfigForwardSoftLimitEnable(feedbackConfig.forwardSwitch);

  gearRatio = feedbackConfig.gearRatio;
  tolerance = pidConfig.tolerance;
  this->profileConfig = profileConfig;
}

void FalconVelocityController::setVelocity(
    units::radians_per_second_t velocity) {
  RawVelocityUnit_t targetVelocity(velocity);

  if (velocity > profileConfig.maxVelocity) {
    targetVelocity = profileConfig.maxVelocity;
  } else if (velocity < profileConfig.minVelocity) {
    targetVelocity = profileConfig.minVelocity;
  }

  motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity,
                      (targetVelocity * gearRatio)());
}

units::radians_per_second_t
FalconVelocityController::getTargetVelocity() const {
  return RawVelocityUnit_t(motorcontroller.GetClosedLoopTarget() / gearRatio);
}

units::radians_per_second_t
FalconVelocityController::getVelocity() const {
  return RawVelocityUnit_t(motorcontroller.GetSelectedSensorVelocity() / gearRatio);
}

units::radians_per_second_t FalconVelocityController::getTolerance() const {
  return tolerance;
}

void FalconVelocityController::setPower(double power) {
  motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                      power);
}

void FalconVelocityController::disable() { motorcontroller.Disable(); }

void FalconVelocityController::stop() { motorcontroller.StopMotor(); }

units::radian_t FalconVelocityController::getPosition() const {
  return RawPositionUnit_t(motorcontroller.GetSelectedSensorPosition() / gearRatio);
}

void FalconVelocityController::zeroPosition(units::radian_t offset) {
  motorcontroller.SetSelectedSensorPosition(
      RawPositionUnit_t(offset * gearRatio)());
}

} // namespace rmb
