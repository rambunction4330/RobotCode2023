#include "ManipulatorSubsystem.h"
#include "manipulator/ManipulatorConstants.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include <iostream>

ManipulatorSubsystem::ManipulatorSubsystem() {
  // Zero Arm
  armMotor->zeroPosition(95_deg);
  setArmAngle(ManipulatorConstants::Arm::range.maxPosition);
  setArmAngle(ManipulatorConstants::Elevator::range.minPosition);

  // Zero elevator
  elevatorMotor->zeroPosition(ManipulatorConstants::Elevator::minHeight);
  setElevatorHeight(ManipulatorConstants::Elevator::minHeight);
}

void ManipulatorSubsystem::Periodic() {
  // Continualy set arm position to it's target so that it will the dynamic 
  // limits again if teh elevator has moved.
  setArmAngle(armMotor->getTargetPosition());
  // std::cout << "Target: " << units::angle 
}

void ManipulatorSubsystem::setArmAngle(units::radian_t angle) {
  // Clamp to dynamic range based on elevator position;
  units::radian_t clamped = std::clamp(
    angle, calculateArmMinPose(), calculateArmMaxPose()
  );

  armMotor->setPosition(angle);
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

frc2::CommandPtr ManipulatorSubsystem::manualZeroArmCommand(frc2::CommandPS4Controller& controller) {
  return frc2::FunctionalCommand([this] {
    armMotor->zeroPosition(ManipulatorConstants::Arm::range.minPosition);
    armMotor->setPosition(ManipulatorConstants::Arm::range.minPosition);
  }, [this, &controller] () {
    double leftY = std::abs(controller.GetRawAxis(1)) < 0.05 ? 0.0 : controller.GetRawAxis(1);
    incArmAngle(1.0_deg * -leftY);
  }, [this](bool) {
    armMotor->zeroPosition(ManipulatorConstants::Arm::range.maxPosition + 5.0_deg);
    armMotor->setPosition(ManipulatorConstants::Arm::range.maxPosition);
  }, []() {
    return false;
  }).ToPtr();
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
  units::meter_t max = ManipulatorConstants::Elevator::maxHeight;
  units::meter_t height = min + (heightPercentage * (max-min));

 setElevatorHeight(height);
}

units::meter_t ManipulatorSubsystem::getElevatorHeight() const {
  return elevatorMotor->getPosition();
}

units::meter_t ManipulatorSubsystem::getTargetElevatorHeight() const {
  return elevatorMotor->getTargetPosition();
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
