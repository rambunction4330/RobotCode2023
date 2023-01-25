#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include <rmb/motorcontrol/sparkmax/SparkmaxPositionController.h>

namespace ManipulatorConstants {

namespace Elevator {
  rmb::SparkMaxPositionController::MotorConfig leader {
    21, rev::CANSparkMax::MotorType::kBrushless, false
  };

  rmb::SparkMaxPositionController::MotorConfig follower {
    22, rev::CANSparkMax::MotorType::kBrushless, true
  };

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    0.0 /*  <- P */ , 0.0/* <- I */, 0.0 /* <- D */, 0.0 /* <- FF */,
    0.0_rad_per_s /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    1.0 /* <- maxOutput */, -1.0 /* <- minOutput */
  };

  const units::meter_t sproketDiameter = 1.625_in;

  const rmb::SparkMaxPositionController::Range range {
    0.0_m * (2_rad / sproketDiameter)/* <- max */, 1.0_m * (2_rad / sproketDiameter)/* <- min */,
    false /* <- isContinouse */
  };

  const rmb::SparkMaxPositionController::ProfileConfig profileConfig {
    false /* <- useSmartMotion */, 
    0.0_mps * (2_rad / sproketDiameter) /* <- maxVelocity */, 0.0_rad_per_s /* <- minVelocity */,
    0.0_mps_sq * (2_rad / sproketDiameter)  /* <- maxAcceleration */,
    rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal /* <- accelStrategy */
  };

  const rmb::SparkMaxPositionController::FeedbackConfig feedbackConfig {
    4.0 /* <- gearRatio */, 
    rmb::SparkMaxPositionController::EncoderType::HallSensor/* <- encoderType */,
    42 /* <- countPerRev */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- forwardSwitch */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- reverseSwitch */
  };
}

namespace Arm {
  rmb::SparkMaxPositionController::MotorConfig motorConfig {
    31, rev::CANSparkMax::MotorType::kBrushless, false
  };

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    0.0 /*  <- P */ , 0.0/* <- I */, 0.0 /* <- D */, 0.0 /* <- FF */,
    0.0_rad_per_s /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    1.0 /* <- maxOutput */, -1.0 /* <- minOutput */
  };

  const rmb::SparkMaxPositionController::Range range {
    0.0_rad /* <- max */, 1.0_rad /* <- min */,
    false /* <- isContinouse */
  };

  const rmb::SparkMaxPositionController::ProfileConfig profileConfig {
    false /* <- useSmartMotion */, 
    0.0_rpm /* <- maxVelocity */, 0.0_rad_per_s /* <- minVelocity */,
    0.0_rad_per_s_sq  /* <- maxAcceleration */,
    rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal /* <- accelStrategy */
  };

  const rmb::SparkMaxPositionController::FeedbackConfig feedbackConfig {
    4.0 /* <- gearRatio */, 
    rmb::SparkMaxPositionController::EncoderType::HallSensor/* <- encoderType */,
    42 /* <- countPerRev */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- forwardSwitch */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- reverseSwitch */
  };
}

namespace Claw {

}

}