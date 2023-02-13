#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include <rmb/motorcontrol/sparkmax/SparkmaxPositionController.h>
#include <rmb/motorcontrol/feedforward/ArmFeedforward.h>
#include <rmb/motorcontrol/feedforward/ElevatorFeedforward.h>

namespace ManipulatorConstants {

const units::inch_t minClawHeight = 6_in;
const units::inch_t maxClawHeight = 75_in;

namespace Elevator {
  const rmb::SparkMaxPositionController::MotorConfig leader {
    22, rev::CANSparkMax::MotorType::kBrushless, true
  };

  const rmb::SparkMaxPositionController::MotorConfig follower {
    21, rev::CANSparkMax::MotorType::kBrushless, true
  };

  const units::meter_t sproketDiameter = 1.175_in;

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    0.08 /*  <- P */ , 0.0/* <- I */, 1.0 /* <- D */, 0.0 /* <- FF */,
    3.0_in * (2_rad / sproketDiameter) /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    0.5 /* <- maxOutput */, -0.5 /* <- minOutput */
  };

  const std::shared_ptr<rmb::Feedforward<units::radians>> feedforward {
    std::make_shared<rmb::ElevatorFeedforward<units::radians>>(
      rmb::ElevatorFeedforward<units::radians>::Ks_t{0.0} /* <- Ks */, 
      rmb::ElevatorFeedforward<units::radians>::Ks_t{0.5} /* <- Kg */, 
      rmb::ElevatorFeedforward<units::radians>::Kv_t{0.0} /* <- Kv */, 
      rmb::ElevatorFeedforward<units::radians>::Ka_t{0.0} /* <- Ka */)
  };

  const rmb::SparkMaxPositionController::Range range {
    11_in * (2_rad / sproketDiameter)/* <- min */,
    38_in * (2_rad / sproketDiameter)/* <- max */,
    false /* <- isContinouse */
  };

  const rmb::SparkMaxPositionController::ProfileConfig profileConfig {
    false /* <- useSmartMotion */, 
    0.2_mps * (2_rad / sproketDiameter) /* <- maxVelocity */, 0.0_rad_per_s /* <- minVelocity */,
    0.4_mps_sq * (2_rad / sproketDiameter)  /* <- maxAcceleration */,
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
  const rmb::SparkMaxPositionController::MotorConfig motorConfig {
    31, rev::CANSparkMax::MotorType::kBrushless, false
  };

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    0.05 /*  <- P */ , 0.0 /* <- I */, 1.0 /* <- D */, 0.0 /* <- FF */,
    3.0_deg /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    0.4 /* <- maxOutput */, -0.3 /* <- minOutput */
  };

  const std::shared_ptr<rmb::Feedforward<units::radians>> feedforward {
    std::make_shared<rmb::ArmFeedforward>(rmb::ArmFeedforward::Ks_t{0.0} /* <- Ks */, 
                                          rmb::ArmFeedforward::Ks_t{0.75} /* <- Kcos */, 
                                          rmb::ArmFeedforward::Kv_t{0.0} /* <- Kv */, 
                                          rmb::ArmFeedforward::Ka_t{0.0} /* <- Ka */)
  };

  const rmb::SparkMaxPositionController::Range range {
    -50_deg /* <- min */, 92.5_deg /* <- max */,
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

  const units::meter_t length = 44_in;
} // namespace Arm
}