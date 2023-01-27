#include "ManipulatorSubsystem.h"

ManipulatorSubsystem::ManipulatorSubsystem() {
    armMotor->zeroPosition(100_deg);
    elevatorMotor->zeroPosition(
        ManipulatorConstants::Elevator::range.minPosition);
}

void ManipulatorSubsystem::Periodic() {
    setArmPosition(armMotor->getTargetPosition());
}

// begin arm
void ManipulatorSubsystem::setArmPosition(units::radian_t position) {
    // Clamp to dynamic range based on elevator position;
    units::radian_t clamped =
        std::clamp(position, calculateArmMinPose(), calculateArmMaxPose());
    armMotor->setPosition(clamped);
}

void ManipulatorSubsystem::incArmPositon(units::radian_t increment) {
    setArmPosition(armMotor->getTargetPosition() + increment);
}

units::radian_t ManipulatorSubsystem::getArmPosition() const {
    return armMotor->getPosition();
}

units::radian_t ManipulatorSubsystem::getArmError() const {
    return armMotor->getError();
}

bool ManipulatorSubsystem::armAtPosition() const { return armMotor->atTarget(); }

bool ManipulatorSubsystem::armIsRaising() const {
    return !armAtPosition() && getArmError() < 0.0_rad;
}

bool ManipulatorSubsystem::armIsLowering() const {
    return !armAtPosition() && getArmError() > 0.0_rad;
}

units::radian_t ManipulatorSubsystem::calculateArmMinPose() const {
    // return -std::numeric_limits<units::radian_t>::infinity();
    return units::math::asin(getElevatorHeight() / ManipulatorConstants::Arm::length);
}

units::radian_t ManipulatorSubsystem::calculateArmMaxPose() const {
    // return std::numeric_limits<units::radian_t>::infinity();
    return units::math::asin((ManipulatorConstants::maxHeight - getElevatorHeight()) / ManipulatorConstants::Arm::length);
}
// end arm

// begin elevator
void ManipulatorSubsystem::setElevatorHeightPercent(double heightPercentage) {
    elevatorMotor->setPosition(heightPercentage * ManipulatorConstants::Elevator::range.maxPosition);
}

void ManipulatorSubsystem::setElevatorHeight(units::meter_t height) {
    elevatorMotor->setPosition(height);
}

// double ManipulatorSubsystem::getElevatorHeightPercent() {
//     return elevatorMotor->getPosition() / ManipulatorConstants::Elevator::range.maxPosition;
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