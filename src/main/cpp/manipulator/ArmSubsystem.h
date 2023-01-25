// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rmb/motorcontrol/feedback/AngularPositionFeedbackController.h>

#include "manipulator/ManipulatorConstants.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  std::shared_ptr<rmb::AngularPositionFeedbackController> elevatorMotor {
    std::make_shared<rmb::SparkMaxPositionController>(
      ManipulatorConstants::Arm::motorConfig, ManipulatorConstants::Arm::pidConfig,
      ManipulatorConstants::Arm::range, ManipulatorConstants::Arm::profileConfig, 
      ManipulatorConstants::Arm::feedbackConfig
    )
  };
};
