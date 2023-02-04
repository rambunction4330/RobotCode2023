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

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    0.05 /*  <- P */ , 0.0/* <- I */, 0.2 /* <- D */, 0.0 /* <- FF */,
    0.0_rad /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    0.4 /* <- maxOutput */, -0.3 /* <- minOutput */
  };

  const std::shared_ptr<rmb::Feedforward<units::radians>> feedforward {
    std::make_shared<rmb::ElevatorFeedforward<units::radians>>(
      rmb::ElevatorFeedforward<units::radians>::Ks_t{0.0} /* <- Ks */, 
      rmb::ElevatorFeedforward<units::radians>::Ks_t{0.47} /* <- Kg */, 
      rmb::ElevatorFeedforward<units::radians>::Kv_t{0.0} /* <- Kv */, 
      rmb::ElevatorFeedforward<units::radians>::Ka_t{0.0} /* <- Ka */)
  };

  const units::meter_t sproketDiameter = 1.175_in;

  const rmb::SparkMaxPositionController::Range range {
    12_in * (2_rad / sproketDiameter)/* <- min */,
    39_in * (2_rad / sproketDiameter)/* <- max */,
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
    0.05 /*  <- P */ , 0.0 /* <- I */, 5.0 /* <- D */, 0.0 /* <- FF */,
    0.0_rad /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    0.2 /* <- maxOutput */, -0.5 /* <- minOutput */
  };

  const std::shared_ptr<rmb::Feedforward<units::radians>> feedforward {
    std::make_shared<rmb::ArmFeedforward>(rmb::ArmFeedforward::Ks_t{0.0} /* <- Ks */, 
                                          rmb::ArmFeedforward::Ks_t{0.5} /* <- Kcos */, 
                                          rmb::ArmFeedforward::Kv_t{0.0} /* <- Kv */, 
                                          rmb::ArmFeedforward::Ka_t{0.0} /* <- Ka */)
  };

  const rmb::SparkMaxPositionController::Range range {
    -90_deg /* <- min */, 90_deg /* <- max */,
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

namespace Claw {
  const rmb::SparkMaxPositionController::MotorConfig motorConfig {
    41, rev::CANSparkMax::MotorType::kBrushed, false
  };

  const rmb::SparkMaxPositionController::PIDConfig pidConfig {
    10.0 /*  <- P */ , 0.0 /* <- I */, 2.0 /* <- D */, 0.0 /* <- FF */,
    0.0_rad /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    0.6 /* <- maxOutput */, -0.2 /* <- minOutput */
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

}