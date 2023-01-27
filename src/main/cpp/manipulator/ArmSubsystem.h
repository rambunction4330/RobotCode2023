// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angular_velocity.h>

#include <frc2/command/SubsystemBase.h>

#include <rmb/motorcontrol/feedback/AngularPositionFeedbackController.h>

#include "manipulator/ManipulatorConstants.h"
#include "manipulator/ElevatorSubsystem.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem(const ElevatorSubsystem& elevatorSubsystem);

  void setArmPosition(units::radian_t position);
  void incArmPositon(units::radian_t increment);

  units::radian_t getArmPosition() const;

  units::radian_t getError() const;
  bool atPosition() const;
  bool isRaising() const;
  bool isLowering() const;

  void Periodic() override;

 private:

  // TODO: Dynamic Arm Range
  units::radian_t calculateMinPose() const;
  units::radian_t calculateMaxPose() const;

  std::shared_ptr<rmb::AngularPositionFeedbackController> armMotor {
    std::make_shared<rmb::SparkMaxPositionController>(
      ManipulatorConstants::Arm::motorConfig, ManipulatorConstants::Arm::pidConfig,
      ManipulatorConstants::Arm::range, ManipulatorConstants::Arm::profileConfig, 
      ManipulatorConstants::Arm::feedbackConfig
    )
  };

  const ElevatorSubsystem& elevatorSubsystem;
};
