// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/DriveSubsystem.h"

#include <cmath>
#include <iostream>

#include <frc2/command/RamseteCommand.h>
#include <frc2/command/RunCommand.h>

#include "drivetrain/commands/BalanceCommand.h"

DriveSubsystem::DriveSubsystem() {
  // Show the robots poition on the feild in shuffleboard.
  frc::SmartDashboard::PutData("Feild", &displayFeild);
}

void DriveSubsystem::arcadeDrive(double xSpeed, double zRotation) {
  drive.arcadeDrive(xSpeed, zRotation);
}

void DriveSubsystem::arcadeDrive(const rmb::LogitechJoystick &joystick) {
  arcadeDrive(joystick.GetX() * joystick.GetThrottle(),
              joystick.GetTwist() * joystick.GetThrottle());
}

void DriveSubsystem::arcadeDrive(const frc2::CommandXboxController &gamepad) {
  // Limit to half speed by default, with the left trigger inceasing max speed,
  // and right reducing it.
  double multiplier = 0.5;
  multiplier += gamepad.GetLeftTriggerAxis() / 2.0;
  multiplier -= gamepad.GetRightTriggerAxis() / 4.0;

  arcadeDrive(-wpi::sgn(gamepad.GetLeftY()) * std::pow(gamepad.GetLeftY(), 2) * multiplier,
              -wpi::sgn(gamepad.GetRightX()) * std::pow(gamepad.GetRightX(), 2) * multiplier);
}

frc2::CommandPtr
DriveSubsystem::arcadeDriveCommand(const rmb::LogitechJoystick &joystick) {
  // Wrap drive function as continually executing command.
  return frc2::RunCommand([&]() { arcadeDrive(joystick); }, {this}).ToPtr();
}

frc2::CommandPtr
DriveSubsystem::arcadeDriveCommand(const frc2::CommandXboxController &gamepad) {
  // Wrap drive function as continually executing command.
  return frc2::RunCommand([&]() { arcadeDrive(gamepad); }, {this}).ToPtr();
}

void DriveSubsystem::curvatureDrive(double xSpeed, double zRotation,
                                    bool turnInPlace) {
  drive.curvatureDrive(xSpeed, zRotation, turnInPlace);
}

void DriveSubsystem::curvatureDrive(const rmb::LogitechJoystick &joystick) {
  curvatureDrive(joystick.GetX() * joystick.GetThrottle(),
                 joystick.GetTwist() * joystick.GetThrottle(),
                 joystick.GetTrigger());
}

void DriveSubsystem::curvatureDrive(
    const frc2::CommandXboxController &gamepad) {
  // Limit to half speed by default, with the left trigger inceasing max speed,
  // and right reducing it.
  double multiplier = 0.5;
  multiplier += gamepad.GetLeftTriggerAxis() / 2.0;
  multiplier -= gamepad.GetRightTriggerAxis() / 4.0;

  curvatureDrive(gamepad.GetLeftX() * multiplier,
                 gamepad.GetRightY() * multiplier, gamepad.GetLeftBumper());
}

frc2::CommandPtr
DriveSubsystem::curvatureDriveCommand(const rmb::LogitechJoystick &joystick) {
  // Wrap drive function as continually executing command.
  return frc2::CommandPtr(
      frc2::RunCommand([&]() { curvatureDrive(joystick); }, {this}));
}

frc2::CommandPtr DriveSubsystem::curvatureDriveCommand(
    const frc2::CommandXboxController &gamepad) {
  // Wrap drive function as continually executing command.
  return frc2::RunCommand([&]() { curvatureDrive(gamepad); }, {this}).ToPtr();
}

void DriveSubsystem::tankDrive(double leftSpeed, double rightSpeed) {
  drive.tankDrive(leftSpeed, rightSpeed);
}

void DriveSubsystem::tankDrive(const frc2::CommandXboxController &gamepad) {
  tankDrive(gamepad.GetLeftX(), -gamepad.GetRightX());
}

frc2::CommandPtr
DriveSubsystem::tankDriveCommand(const frc2::CommandXboxController &gamepad) {
  // Wrap drive function as continually executing command.
  return frc2::RunCommand([&]() { tankDrive(gamepad); }, {this}).ToPtr();
}

void DriveSubsystem::resetOdometry(frc::Pose2d pose) {
  odometry.resetPose(pose);
}

frc::Pose2d DriveSubsystem::getPose() const { return odometry.getPose(); }

frc::DifferentialDriveWheelSpeeds DriveSubsystem::getWheelSpeeds() const {
  return {left->getVelocity(), right->getVelocity()};
}

frc::ChassisSpeeds DriveSubsystem::getChassisSpeeds() const {
  return DriveConstants::kinematics.ToChassisSpeeds(getWheelSpeeds());
}

void DriveSubsystem::stop() {
  left->stop();
  right->stop();
}

void DriveSubsystem::driveWheelSpeeds(units::meters_per_second_t left,
                                      units::meters_per_second_t right) {
  drive.driveWheelSpeeds(left, right);
}

void DriveSubsystem::driveWheelSpeeds(
    frc::DifferentialDriveWheelSpeeds speeds) {
  drive.driveWheelSpeeds(speeds);
}

void DriveSubsystem::driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds) {
  drive.driveChassisSpeeds(chassisSpeeds);
}

units::radian_t DriveSubsystem::getRobotPitch() {
  return units::degree_t(gyro->GetRoll());
}

frc2::CommandPtr DriveSubsystem::getBalanceCommand() {
  // Get balance command for this drivetrain.
  return frc2::CommandPtr(BalanceCommand(*this));
}

void DriveSubsystem::Periodic() {
  // Update odometry while sending the new positon to  shuffleboard.
  displayFeild.SetRobotPose(odometry.updatePose());
}
