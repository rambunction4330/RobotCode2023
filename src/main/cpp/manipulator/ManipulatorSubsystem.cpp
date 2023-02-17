#include "ManipulatorSubsystem.h"
#include "manipulator/ManipulatorConstants.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include <iostream>

ManipulatorSubsystem::ManipulatorSubsystem() {
  // Zero Arm
  armMotor->zeroPosition(100_deg);
  setArmAngle(ManipulatorConstants::Arm::range.maxPosition);

  // Zero elevator
  elevatorMotor->zeroPosition(ManipulatorConstants::Elevator::minHeight);
  setElevatorHeight(ManipulatorConstants::Elevator::maxHeight);
}

void ManipulatorSubsystem::Periodic() {
  // Continualy set arm position to it's target so that it will the dynamic 
  // limits again if teh elevator has moved.
  setArmAngle(armMotor->getTargetPosition());
}

void ManipulatorSubsystem::setArmAngle(units::radian_t angle) {
  // Clamp to dynamic range based on elevator position;
  units::radian_t clamped = std::clamp(
    angle, calculateArmMinPose(), calculateArmMaxPose()
  );

  armMotor->setPosition(clamped);
}

void ManipulatorSubsystem::incArmAngle(units::radian_t increment) {
  setArmAngle(armMotor->getTargetPosition() + increment);
}

units::radian_t ManipulatorSubsystem::getArmAngle() const {
  return armMotor->getPosition();
}

units::radian_t ManipulatorSubsystem::getArmError() const {
  return armMotor->getError();
}

bool ManipulatorSubsystem::armAtPosition() const { 
  return armMotor->atTarget(); 
}

bool ManipulatorSubsystem::armIsRaising() const {
  // Check movement direction through the sign of error as long as it is not 
  // at the target.
  return !armAtPosition() && getArmError() < 0.0_rad;
}

bool ManipulatorSubsystem::armIsLowering() const {
  // Check movement direction through the sign of error as long as it is not 
  // at the target.
  return !armAtPosition() && getArmError() > 0.0_rad;
}

units::radian_t ManipulatorSubsystem::calculateArmMinPose() const {
  // Find hypotinus and side oposite of angle
  units::meter_t o = getElevatorHeight() - ManipulatorConstants::minClawHeight;
  units::meter_t h =  ManipulatorConstants::Arm::length;

  // Determin angle
  return -units::math::asin(o/h);
}

units::radian_t ManipulatorSubsystem::calculateArmMaxPose() const {
  // Find hypotinus and side oposite of angle
  units::meter_t o = ManipulatorConstants::maxClawHeight - getElevatorHeight();
  units::meter_t h =  ManipulatorConstants::Arm::length;

  // Determin angle
  return units::math::asin(o/h);
}

void ManipulatorSubsystem::setElevatorHeight(units::meter_t height) {
  elevatorMotor->setPosition(height);
}

void ManipulatorSubsystem::setElevatorHeightPercent(double heightPercentage) {
  // Calculate height from  percentage of total range.
  units::meter_t min = ManipulatorConstants::Elevator::minHeight;
  units::meter_t max = ManipulatorConstants::Elevator::minHeight;
  units::meter_t height = min + (heightPercentage * (max-min));

  elevatorMotor->setPosition(height);
}

units::meter_t ManipulatorSubsystem::getElevatorHeight() const {
  return elevatorMotor->getPosition();
}

units::meter_t ManipulatorSubsystem::getElevatorError() const {
  return elevatorMotor->getError();
}

bool ManipulatorSubsystem::elevatorAtHeight() {
  return elevatorMotor->atTarget();
}

bool ManipulatorSubsystem::elevatorIsRaising() {
  // Check movement direction through the sign of error as long as it is not 
  // at the target.
  return !elevatorAtHeight() && getElevatorError() < 0.0_m;
}

bool ManipulatorSubsystem::elevatorIsLowering() {
    // Check movement direction through the sign of error as long as it is not 
  // at the target.
  return !elevatorAtHeight() && getElevatorError() > 0.0_m;
}
// end elevator

void ManipulatorSubsystem::setState(const ManipulatorState state) {
  // Set both states
  setArmAngle(state.armAngle);
  setElevatorHeight(state.elevatorHeight);
}

frc2::CommandPtr ManipulatorSubsystem::getStateCommand(
  const ManipulatorState state) {

  // Command that sets states and waits for it to reach that state.
  return frc2::FunctionalCommand([this, state](){ setState(state); }, 
                                 [](){}, [](bool){}, 
                                 [this]() { 
                                  return armAtPosition() && elevatorAtHeight(); 
                                 }, {this}).ToPtr();
}

frc2::CommandPtr ManipulatorSubsystem::getInstantStateCommand(
  const ManipulatorState state) {

  // Command that sets states instantly without waiting
  return frc2::InstantCommand([this, state](){ setState(state); }, 
                              {this}).ToPtr();
}