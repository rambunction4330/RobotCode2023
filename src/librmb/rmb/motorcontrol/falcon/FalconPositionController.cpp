#include "FalconPositionController.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "units/angle.h"

namespace ctre {
using namespace phoenix::motorcontrol::can;
}

namespace rmb {

FalconPositionController::FalconPositionController(
    FalconPositionControllerHelper::MotorConfig config,
    FalconPositionControllerHelper::PIDConfig pidConfig,
    FalconPositionControllerHelper::Range range,
    FalconPositionControllerHelper::FeedbackConfig feedbackConfig)
    : motorcontroller(config.id), range(range) {

  motorcontroller.SetInverted(config.inverted);

  motorcontroller.Config_kD(0, pidConfig.d);
  motorcontroller.Config_kI(0, pidConfig.i);
  motorcontroller.Config_kP(0, pidConfig.p);
  motorcontroller.Config_kF(0, pidConfig.ff);
  motorcontroller.ConfigAllowableClosedloopError(0, RawPositionUnit_t(pidConfig.tolerance)());

  motorcontroller.Config_IntegralZone(0, pidConfig.iZone);
  motorcontroller.ConfigMaxIntegralAccumulator(0, pidConfig.iMaxAccumulator);
  motorcontroller.ConfigPeakOutputForward(0, pidConfig.maxOutput);
  motorcontroller.ConfigPeakOutputReverse(0, pidConfig.minOutput);

  motorcontroller.ConfigForwardSoftLimitEnable(feedbackConfig.forwardSwitch);

  gearRatio = feedbackConfig.gearRatio;
  tolerance = pidConfig.tolerance;
}

void FalconPositionController::setPosition(units::radian_t position) {
  RawPositionUnit_t targetPosition(position);

  if(targetPosition > range.maxPosition) {
    targetPosition = range.maxPosition;
  } else if(targetPosition < range.minPosition) {
    targetPosition = range.minPosition;
  }

  motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::Position, (targetPosition * gearRatio)());
}

units::radian_t FalconPositionController::getTargetPosition() const {
  return RawPositionUnit_t(motorcontroller.GetClosedLoopTarget()) / gearRatio;
}

units::radian_t FalconPositionController::getMinPosition() const {
  return range.minPosition;
}

units::radian_t FalconPositionController::getMaxPosition() const {
  return range.maxPosition;
}

void FalconPositionController::disable() {
  motorcontroller.Disable();
}

void FalconPositionController::stop() {
  motorcontroller.StopMotor();
}

units::radians_per_second_t FalconPositionController::getVelocity() const {
  return RawVelocityUnit_t(motorcontroller.GetSelectedSensorVelocity() / gearRatio);
}

units::radian_t FalconPositionController::getPosition() const {
  return RawPositionUnit_t(motorcontroller.GetSelectedSensorPosition() / gearRatio);
}

void FalconPositionController::zeroPosition(units::radian_t offset) {
  motorcontroller.SetSelectedSensorPosition(RawPositionUnit_t(offset * gearRatio)());
}

units::radian_t FalconPositionController::getTolerance() const {
  return tolerance;
}

} // namespace rmb
