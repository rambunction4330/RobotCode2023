#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include <rmb/motorcontrol/sparkmax/SparkmaxPositionController.h>

namespace ManipulatorConstants {

namespace Elevator {
  const rmb::SparkMaxPositionController::MotorConfig leader {
    21, rev::CANSparkMax::MotorType::kBrushless, false
  };

  const rmb::SparkMaxPositionController::MotorConfig follower {
    22, rev::CANSparkMax::MotorType::kBrushless, true
  };

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    0.3 /*  <- P */ , 0.0001/* <- I */, 1.5 /* <- D */, 0.0 /* <- FF */,
    0.0_rad /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    0.3 /* <- maxOutput */, -0.2 /* <- minOutput */
  };

  const units::meter_t sproketDiameter = 1.375_in;

  const rmb::SparkMaxPositionController::Range range {
    27.25_in * (2_rad / sproketDiameter)/* <- max */, 0.0_in * (2_rad / sproketDiameter)/* <- min */,
    false /* <- isContinouse */
  };

  const rmb::SparkMaxPositionController::ProfileConfig profileConfig {
    false /* <- useSmartMotion */, 
    0.2_mps * (2_rad / sproketDiameter) /* <- maxVelocity */, 0.0_rad_per_s /* <- minVelocity */,
    0.4_mps_sq * (2_rad / sproketDiameter)  /* <- maxAcceleration */,
    rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal /* <- accelStrategy */
  }; 

  const rmb::SparkMaxPositionController::FeedbackConfig feedbackConfig {
    5.0 /* <- gearRatio */, 
    rmb::SparkMaxPositionController::EncoderType::HallSensor/* <- encoderType */,
    42 /* <- countPerRev */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- forwardSwitch */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- reverseSwitch */
  };
}

namespace Arm {
  const rmb::SparkMaxPositionController::MotorConfig motorConfig {
    31, rev::CANSparkMax::MotorType::kBrushless, false
  };

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    0.9 /*  <- P */ , 0.01/* <- I */, 1.5 /* <- D */, 0.0 /* <- FF */,
    0.0_rad /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    0.75 /* <- maxOutput */, -0.75 /* <- minOutput */
  };

  const rmb::SparkMaxPositionController::Range range {
    100_deg /* <- max */, -10_deg /* <- min */,
    false /* <- isContinouse */
  };

  const rmb::SparkMaxPositionController::ProfileConfig profileConfig {
    false /* <- useSmartMotion */, 
    0.0_rpm /* <- maxVelocity */, 0.0_rad_per_s /* <- minVelocity */,
    0.0_rad_per_s_sq  /* <- maxAcceleration */,
    rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal /* <- accelStrategy */
  };

  const rmb::SparkMaxPositionController::FeedbackConfig feedbackConfig {
    45.0 * (54.0 / 22.0) /* <- gearRatio */, 
    rmb::SparkMaxPositionController::EncoderType::HallSensor/* <- encoderType */,
    42 /* <- countPerRev */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- forwardSwitch */,
    rmb::SparkMaxPositionController::LimitSwitchConfig::Disabled /* <- reverseSwitch */
  };
}

namespace Claw {

}

}