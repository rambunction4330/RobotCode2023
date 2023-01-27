// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "manipulator/ArmSubsystem.h"
#include <iostream>

ArmSubsystem::ArmSubsystem(const ElevatorSubsystem& elevator) : elevatorSubsystem(elevator) {
  armMotor->zeroPosition(100_deg);
}

void ArmSubsystem::setArmPosition(units::radian_t position) {
  // Clamp to dynamic range based on elevator position;
  units::radian_t clamped = std::clamp(position, calculateMinPose(), calculateMaxPose());
  armMotor->setPosition(clamped);
}

void ArmSubsystem::incArmPositon(units::radian_t increment) {
  setArmPosition(armMotor->getTargetPosition() + increment);
}

units::radian_t ArmSubsystem::getArmPosition() const {
  return armMotor->getPosition();
}

units::radian_t ArmSubsystem::getError() const {
  return armMotor->getError();
}

bool ArmSubsystem::atPosition() const {
  return armMotor->atTarget();
}

bool ArmSubsystem::isRaising() const {
  return !atPosition() && getError() < 0.0_rad;
}

bool ArmSubsystem::isLowering() const {
  return !atPosition() && getError() > 0.0_rad;
}

units::radian_t ArmSubsystem::calculateMinPose() const {
  return -std::numeric_limits<units::radian_t>::infinity();
}

units::radian_t ArmSubsystem::calculateMaxPose() const {
  return std::numeric_limits<units::radian_t>::infinity();
}

void ArmSubsystem::Periodic() {
  // Set position to target aragin in order to check dynamic bounds.
  ArmSubsystem::setArmPosition(armMotor->getTargetPosition());
}
