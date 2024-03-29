// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/DriveSubsystem.h"

#include <cmath>
#include <iostream>

#include <frc/filter/LinearFilter.h>

#include <frc2/command/RamseteCommand.h>
#include <frc2/command/RunCommand.h>

#include "drivetrain/commands/BalanceCommand.h"
#include "units/acceleration.h"
#include "units/length.h"
#include "units/math.h"
#include "units/velocity.h"

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
  double leftY = abs(gamepad.GetLeftY()) > 0.05 ? gamepad.GetLeftY() : 0.0;
  double rightX = abs(gamepad.GetRightX()) > 0.05 ? gamepad.GetRightX() : 0.0;

  arcadeDrive(-wpi::sgn(leftY) * std::pow(leftY, 2) * multiplier,
              -wpi::sgn(rightX) * std::pow(rightX, 2) * multiplier);
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
  drive.resetPose(pose);
}

units::meters_per_second_squared_t DriveSubsystem::getAcceleration() const {
  auto accelX_sq = units::math::pow<2>(units::standard_gravity_t(gyro->GetWorldLinearAccelX()));
  auto accelY_sq = units::math::pow<2>(units::standard_gravity_t(gyro->GetWorldLinearAccelY()));
  auto accelZ_sq = units::math::pow<2>(units::standard_gravity_t(gyro->GetWorldLinearAccelZ()));

  return units::math::sqrt(accelX_sq + accelY_sq + accelZ_sq);
}

frc::Pose2d DriveSubsystem::getPose() const { return drive.getPose(); }

frc::DifferentialDriveWheelSpeeds DriveSubsystem::getWheelSpeeds() const {
  return drive.getWheelSpeeds();
}

frc::ChassisSpeeds DriveSubsystem::getChassisSpeeds() const {
  return DriveConstants::kinematics.ToChassisSpeeds(getWheelSpeeds());
}

void DriveSubsystem::stop() {
  drive.tankDrive(0.0, 0.0);
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

frc2::CommandPtr DriveSubsystem::getAutoCommand(
    std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap) {
  return drive.fullPPAuto(trajectoryGroup, eventMap, {this});
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
  displayFeild.SetRobotPose(drive.updatePose());
}
