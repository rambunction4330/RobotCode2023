
#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include <rmb/motorcontrol/sparkmax/SparkmaxPositionController.h>
#include <rmb/motorcontrol/feedforward/ArmFeedforward.h>
#include <rmb/motorcontrol/feedforward/ElevatorFeedforward.h>

namespace ClawConstants {
  const rmb::SparkMaxPositionController::MotorConfig motorConfig {
    41, rev::CANSparkMax::MotorType::kBrushed, false
  };

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    10.0 /*  <- P */ , 0.0 /* <- I */, 0.0 /* <- D */, 0.0 /* <- FF */,
    0.0_rad /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    0.4 /* <- maxOutput */, -0.1 /* <- minOutput */
  }; 

  const std::shared_ptr<rmb::Feedforward<units::radians>> feedforward {
    std::make_shared<rmb::SimpleFeedforward<units::radians>>()
  };

  const rmb::SparkMaxPositionController::Range range {
    10_deg /* <- min */, 90_deg /* <- max */,
    false /* <- isContinouse */
  };

  const rmb::SparkMaxPositionController::ProfileConfig profileConfig {
    false /* <- useSmartMotion */, 
    0.0_rpm /* <- maxVelocity */, 0.0_rad_per_s /* <- minVelocity */,
    0.0_rad_per_s_sq  /* <- maxAcceleration */,
    rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal /* <- accelStrategy */
  };

  const rmb::SparkMaxPositionController::FeedbackConfig feedbackConfig {
    1.0 /* <- gearRatio */, 
    rmb::SparkMaxPositionController::EncoderType::Quadrature/* <- encoderType */,
    4096 /* <- countPerRev */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- forwardSwitch */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- reverseSwitch */
  };
}