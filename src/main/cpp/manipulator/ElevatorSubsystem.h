// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <memory>

#include <rmb/motorcontrol/feedback/AngularPositionFeedbackController.h>
#include <rmb/motorcontrol/feedback/LinearPositionFeedbackController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

#include "manipulator/ManipulatorConstants.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  std::shared_ptr<rmb::LinearPositionFeedbackController> elevatorMotor {
    rmb::asLinear(
      (std::shared_ptr<rmb::AngularPositionFeedbackController>)
      std::make_shared<rmb::SparkMaxPositionController>(
        ManipulatorConstants::Elevator::leader, ManipulatorConstants::Elevator::pidConfig,
        ManipulatorConstants::Elevator::range, ManipulatorConstants::Elevator::profileConfig, 
        ManipulatorConstants::Elevator::feedbackConfig, 
        std::initializer_list<const rmb::SparkMaxPositionController::MotorConfig>{ManipulatorConstants::Elevator::follower}
      ),
      ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad
    )
  };
};
