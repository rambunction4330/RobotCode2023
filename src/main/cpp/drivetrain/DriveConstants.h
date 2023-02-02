
#pragma once

#include <frc/controller/RamseteController.h>

#include <AHRS.h>

#include <rmb/motorcontrol/feedback/LinearVelocityFeedbackController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>
#include <rmb/drive/DifferentialDrive.h>
#include <rmb/drive/DifferentialOdometry.h>

namespace DriveConstants {

  const rmb::SparkMaxVelocityController::MotorConfig leftLeader {
    1, rev::CANSparkMax::MotorType::kBrushless, false
  };

  const rmb::SparkMaxVelocityController::MotorConfig leftFollower {
    2, rev::CANSparkMax::MotorType::kBrushless, false
  };

  const rmb::SparkMaxVelocityController::MotorConfig rightLeader {
    3, rev::CANSparkMax::MotorType::kBrushless, true
  };

  const rmb::SparkMaxVelocityController::MotorConfig rightFollower {
    4, rev::CANSparkMax::MotorType::kBrushless, false
  };

  const rmb::SparkMaxVelocityController::PIDConfig pidConfig {
    0.0000015 /*  <- P */ , 0.000000001 /* <- I */, 0.00001 /* <- D */, 0.000184 /* <- FF */,
    0.0_rad_per_s /* <- Tolerance */, 
    0.0 /* <- iZone */, 0.0 /* <- iMaxAccumulator */,
    1.0 /* <- maxOutput */, -1.0 /* <- minOutput */
  };

  const units::meter_t wheelDiameter = 6_in;

  const rmb::SparkMaxVelocityController::ProfileConfig profileConfig {
    true /* <- useSmartMotion */, 
    3.75_mps * (2_rad / wheelDiameter) /* <- maxVelocity */, 0.0_rad_per_s /* <- minVelocity */,
    1.0_mps_sq * (2_rad / wheelDiameter)  /* <- maxAcceleration */,
    rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal /* <- accelStrategy */
  };

  const rmb::SparkMaxVelocityController::FeedbackConfig feedbackConfig {
    10.71 /* <- gearRatio */, 
    rmb::SparkMaxVelocityController::EncoderType::HallSensor/* <- encoderType */,
    42 /* <- countPerRev */,
    rmb::SparkMaxVelocityController::LimitSwitchConfig::Disabled /* <- forwardSwitch */,
    rmb::SparkMaxVelocityController::LimitSwitchConfig::Disabled /* <- reverseSwitch */
  };

  static const frc::RamseteController ramseteController {
    units::unit_t<frc::RamseteController::b_unit>(2.0) /* <- b */,
    units::unit_t<frc::RamseteController::zeta_unit>(0.7) /* <- zeta */ 
  };

  static frc::DifferentialDriveKinematics kinematics{ 27.75_in /* <- track width */};

  const frc::SerialPort::Port gyroPort = frc::SerialPort::Port::kMXP;

  const static std::string visionTableString = "visionTable";
}