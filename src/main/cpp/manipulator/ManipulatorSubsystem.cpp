#include "ManipulatorSubsystem.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/FunctionalCommand.h>

#include <iostream>

ManipulatorSubsystem::ManipulatorSubsystem() {
    armMotor->zeroPosition(95_deg);
    setArmPosition(92.5_deg);
    elevatorMotor->zeroPosition(minElevatorHeight);
    setElevatorHeight(12_in);
}

void ManipulatorSubsystem::Periodic() {
    setArmPosition(targetArmPose);
}

// begin arm
void ManipulatorSubsystem::setArmPosition(units::radian_t position) {
    // Clamp to dynamic range based on elevator position;
    units::radian_t clamped = std::clamp(position, calculateArmMinPose(), calculateArmMaxPose());
    
    targetArmPose = clamped;
    frc::TrapezoidProfile<units::radians> profile {{100_deg_per_s, 200_deg_per_s_sq}, 
                                                   {targetArmPose, 0.0_rad_per_s},
                                                   {armMotor->getPosition(), armMotor->getVelocity()}};

    armMotor->setPosition(profile.Calculate(200_ms).position);
}

void ManipulatorSubsystem::incArmPositon(units::radian_t increment) {
    setArmPosition(targetArmPose + increment);
}

units::radian_t ManipulatorSubsystem::getArmPosition() const {
    return armMotor->getPosition();
}

units::radian_t ManipulatorSubsystem::getArmError() const {
    return armMotor->getPosition() - targetArmPose;
}

bool ManipulatorSubsystem::armAtPosition() const { return units::math::abs(getArmError()) < ManipulatorConstants::Arm::pidConfig.tolerance; }

bool ManipulatorSubsystem::armIsRaising() const {
    return !armAtPosition() && getArmError() < 0.0_rad;
}

bool ManipulatorSubsystem::armIsLowering() const {
    return !armAtPosition() && getArmError() > 0.0_rad;
}

units::radian_t ManipulatorSubsystem::calculateArmMinPose() const {
    // return -std::numeric_limits<units::radian_t>::infinity();
    return -units::math::asin((getElevatorHeight() - ManipulatorConstants::minClawHeight) / ManipulatorConstants::Arm::length);
}

units::radian_t ManipulatorSubsystem::calculateArmMaxPose() const {
    // return std::numeric_limits<units::radian_t>::infinity();
    return units::math::asin((ManipulatorConstants::maxClawHeight - getElevatorHeight()) / ManipulatorConstants::Arm::length);
}
// end arm

// begin elevator
void ManipulatorSubsystem::setElevatorHeightPercent(double heightPercentage) {
    elevatorMotor->setPosition(minElevatorHeight  + (heightPercentage * (maxElevatorHeight - minElevatorHeight)));
}

// void ManipulatorSubsystem::setElevatorHeight(units::meter_t height) {
//     elevatorMotor->setPosition(height);
// }

// double ManipulatorSubsystem::getElevatorHeightPercent() {
//     return elevatorMotor->getPosition() - ManipulatorConstants::Elevator::range.minPosition / ManipulatorConstants::Elevator::range.maxPosition;
// }

units::meter_t ManipulatorSubsystem::getElevatorHeight() const {
    return elevatorMotor->getPosition();
}

void ManipulatorSubsystem::setElevatorHeight(units::meter_t height) {
    elevatorMotor->setPosition(height);
}

bool ManipulatorSubsystem::elevatorAtHeight() {
    return elevatorMotor->atTarget();
}

bool ManipulatorSubsystem::elevatorGoingDown() {
    return elevatorMotor->getError() > (elevatorMotor->getTargetPosition() - elevatorMotor->getTolerance());
}

bool ManipulatorSubsystem::elevatorGoingUp() {
    return elevatorMotor->getError() < (elevatorMotor->getTargetPosition() + elevatorMotor->getTolerance());
}
// end elevator

void ManipulatorSubsystem::setState(const ManipulatorState state) {
  setArmPosition(state.armAngle);
  setElevatorHeight(state.elevatorHeight);
}

frc2::CommandPtr ManipulatorSubsystem::getStateCommand(const ManipulatorState state) {
  return frc2::FunctionalCommand([](){}, [this, state](){ setState(state); }, [](bool){}, 
                                 [this]() { return armAtPosition() && elevatorAtHeight(); },
                                 {this}).ToPtr();
}