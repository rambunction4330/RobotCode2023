// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/DriveSubsystem.h"

#include <iostream>

#include <frc2/command/RunCommand.h>
#include <frc2/command/RamseteCommand.h>

#include "drivetrain/commands/BalanceCommand.h"

DriveSubsystem::DriveSubsystem() {
  frc::SmartDashboard::PutData("Feild", &displayFeild);
}

void DriveSubsystem::arcadeDrive(double xSpeed, double zRotation) {
  drive.arcadeDrive(xSpeed, zRotation);
}

void DriveSubsystem::arcadeDrive(const rmb::LogitechJoystick& joystick) {
  arcadeDrive(joystick.GetX() * joystick.GetThrottle(), joystick.GetTwist() * joystick.GetThrottle());
}

void DriveSubsystem::arcadeDrive(const rmb::LogitechGamepad& gamepad) {
  double additional = gamepad.GetLeftTrigger() / 2.0;
  arcadeDrive((0.5 + additional) * gamepad.GetLeftX(), (0.5 + additional) * -gamepad.GetRightY());
}

frc2::CommandPtr DriveSubsystem::arcadeDriveCommand(const rmb::LogitechJoystick& joystick) {
  return frc2::CommandPtr(frc2::RunCommand([&]() { arcadeDrive(joystick); }, {this}));
}

frc2::CommandPtr DriveSubsystem::arcadeDriveCommand(const rmb::LogitechGamepad& gamepad) {
  return frc2::CommandPtr(frc2::RunCommand([&]() { arcadeDrive(gamepad); }, {this}));
}

void DriveSubsystem::curvatureDrive(double xSpeed, double zRotation, bool turnInPlace) {
  drive.curvatureDrive(xSpeed, zRotation, turnInPlace);
}

void DriveSubsystem::curvatureDrive(const rmb::LogitechJoystick& joystick) {
  curvatureDrive(joystick.GetX() * joystick.GetThrottle(), joystick.GetTwist() * joystick.GetThrottle(), joystick.GetTrigger());
}

void DriveSubsystem::curvatureDrive(const rmb::LogitechGamepad& gamepad) {
  curvatureDrive(gamepad.GetLeftX(), gamepad.GetRightY(), gamepad.GetLeftBumper());
}

frc2::CommandPtr DriveSubsystem::curvatureDriveCommand(const rmb::LogitechJoystick& joystick) {
  return frc2::CommandPtr(frc2::RunCommand([&]() { curvatureDrive(joystick); }, {this}));
}

frc2::CommandPtr DriveSubsystem::curvatureDriveCommand(const rmb::LogitechGamepad& gamepad) {
  return frc2::CommandPtr(frc2::RunCommand([&]() { curvatureDrive(gamepad); }, {this}));
}

void DriveSubsystem::tankDrive(double leftSpeed, double rightSpeed) {
  drive.tankDrive(leftSpeed, rightSpeed);
}

void DriveSubsystem::tankDrive(const rmb::LogitechJoystick& left, const rmb::LogitechJoystick& right) {
  tankDrive(left.GetX(), right.GetX());
}

void DriveSubsystem::tankDrive(const rmb::LogitechGamepad& gamepad) {
  tankDrive(gamepad.GetLeftX(), -gamepad.GetRightX());
}

frc2::CommandPtr DriveSubsystem::tankDirveCommand(const rmb::LogitechJoystick& left, const rmb::LogitechJoystick& right) {
  return frc2::CommandPtr(frc2::RunCommand([&]() { tankDrive(left, right); }, {this}));
}

frc2::CommandPtr DriveSubsystem::tankDriveCommand(const rmb::LogitechGamepad& gamepad) {
  return frc2::CommandPtr(frc2::RunCommand([&]() { tankDrive(gamepad); }, {this}));
}

void DriveSubsystem::resetOdometry(frc::Pose2d pose) {
  odometry.resetPose(pose);
}

frc::Pose2d DriveSubsystem::getPose() const {
  return odometry.getPose();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::getWheelSpeeds() const {
  return {left->getVelocity(), right->getVelocity() };
}

frc::ChassisSpeeds DriveSubsystem::getChassisSpeeds() const {
  return DriveConstants::kinematics.ToChassisSpeeds(getWheelSpeeds());
}

void DriveSubsystem::stop() {
  left->stop();
  right->stop();
}

void DriveSubsystem::driveWheelSpeeds(units::meters_per_second_t left, units::meters_per_second_t right) {
  drive.driveWheelSpeeds(left, right);
}

void DriveSubsystem::driveWheelSpeeds(frc::DifferentialDriveWheelSpeeds wheelSpeeds) {
  drive.driveWheelSpeeds(wheelSpeeds);
}

void DriveSubsystem::driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds) {
  drive.driveChassisSpeeds(chassisSpeeds);
}

units::radian_t DriveSubsystem::getRobotPitch() {
  return units::degree_t(gyro->GetRoll());
}

frc2::CommandPtr DriveSubsystem::getBalanceCommand() {
  return frc2::CommandPtr(BalanceCommand(*this));
}

// This method will be called once per scheduler run
void DriveSubsystem::Periodic() {
  displayFeild.SetRobotPose(odometry.updatePose());
}
