#include "rmb/motorcontrol/talon/TalonFXVelocityController.h"

namespace rmb {
TalonFXVelocityController::TalonFXVelocityController(const MotorConfig motorConfig, const PIDConfig pidConfig, 
                            const FeedbackConfig feedbackConfig, std::initializer_list<const MotorConfig> followers,
                            std::function<void(ctre::phoenix::motorcontrol::can::WPI_TalonFX&)> customConfig) :
                            talonFX(motorConfig.id), gearRatio(feedbackConfig.gearRatio) {

  // Set default baseline
  talonFX.ConfigFactoryDefault();

  // Motor Config
  talonFX.SetInverted(motorConfig.inverted);

  // PID Config
  talonFX.Config_kP(0, pidConfig.p);
  talonFX.Config_kI(0, pidConfig.i);
  talonFX.Config_kD(0, pidConfig.d);
  talonFX.Config_kF(0, pidConfig.f);
  talonFX.Config_IntegralZone(0, pidConfig.iZone);
  talonFX.ConfigMaxIntegralAccumulator(0, pidConfig.iMaxAccumulator);
  talonFX.ConfigPeakOutputForward(pidConfig.maxOutput);
  talonFX.ConfigPeakOutputReverse(pidConfig.minOutput);
  talonFX.ConfigClosedLoopPeakOutput(0, pidConfig.maxOutput);

  // Feedback Config
  talonFX.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);

  switch (feedbackConfig.forwardSwitch) {
  case LimitSwitchConfig::NormalyOpen: {
    talonFX.ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                                           ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen);
    break;
  }
  case LimitSwitchConfig::NormalyClosed: {
    talonFX.ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                                           ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyClosed);
    break;
  }
  case LimitSwitchConfig::Disabled: {
    talonFX.ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_Deactivated,
                                           ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled);
    break;
  }
  }

  switch (feedbackConfig.reverseSwitch) {
  case LimitSwitchConfig::NormalyOpen: {
    talonFX.ConfigReverseLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                                           ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen);
    break;
  }
  case LimitSwitchConfig::NormalyClosed: {
    talonFX.ConfigReverseLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                                           ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyClosed);
    break;
  }
  case LimitSwitchConfig::Disabled: {
    talonFX.ConfigReverseLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_Deactivated,
                                           ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled);
    break;
  }
  }
}

void TalonFXVelocityController::setVelocity(units::radians_per_second_t velocity) {
  talonFX.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, EncoderVelocity_t(velocity).to<double>() * gearRatio);
}

units::radians_per_second_t TalonFXVelocityController::getTargetVelocity() const {
  return 0.0_rpm;
}

units::radians_per_second_t TalonFXVelocityController::getMaxVelocity() const {
  return 0.0_rpm;
}

void TalonFXVelocityController::setPower(double power) {
  talonFX.Set(power);
}

void TalonFXVelocityController::disable() {
  talonFX.Disable();
}

void TalonFXVelocityController::stop() {
  talonFX.StopMotor();
}

units::radians_per_second_t TalonFXVelocityController::getVelocity() const {
  auto asConst = const_cast<ctre::phoenix::motorcontrol::can::WPI_TalonFX*>(&talonFX);
  return EncoderVelocity_t((double) asConst->GetSelectedSensorVelocity() / gearRatio);
}

units::radian_t TalonFXVelocityController::getPosition() const {
  auto asConst = const_cast<ctre::phoenix::motorcontrol::can::WPI_TalonFX*>(&talonFX);
  return EncoderTick_t((double) asConst->GetSelectedSensorPosition() / gearRatio);
}

void TalonFXVelocityController::zeroPosition(units::radian_t offset) {
  talonFX.SetSelectedSensorPosition(EncoderTick_t(offset).to<double>() * gearRatio);
}

units::radians_per_second_t TalonFXVelocityController::getTolerance() const {
  return 0.0_rpm;
}
} // namespace rmb