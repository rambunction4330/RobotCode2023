// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "manipulator/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() {
    elevatorMotor->setPosition(ManipulatorConstants::Elevator::range.minPosition);
}

void ElevatorSubsystem::setHeightPercent(double heightPercentage) {
    elevatorMotor->setPosition(heightPercentage * ManipulatorConstants::Elevator::range.maxPosition);
}

void ElevatorSubsystem::setHeight(units::meter_t height) {
    elevatorMotor->setPosition(height);
}

bool ElevatorSubsystem::atHeight() {
    return elevatorMotor->atTarget();
}

bool ElevatorSubsystem::goingDown() {
    return elevatorMotor->getError() > (elevatorMotor->getTargetPosition() - elevatorMotor->getTolerance());
}

bool ElevatorSubsystem::goingUp() {
    return elevatorMotor->getError() < (elevatorMotor->getTargetPosition() + elevatorMotor->getTolerance());
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}
