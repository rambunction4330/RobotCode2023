// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <iostream>

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include <frc2/command/RunCommand.h>

#include <pathplanner/lib/PathPlanner.h>

#include "drivetrain/commands/BalanceCommand.h"

RobotContainer::RobotContainer() {
  // Configure button bindings
  ConfigureBindings();

  // Setup Auto Routines
  for (std::string name : autoNames) {
    // Build Command
    autoCommands.emplace_back(
        autoBuilder.fullAuto(pathplanner::PathPlanner::loadPathGroup(
            name, 2.0_mps, 2.0_mps_sq, true)));

    // Add to chooser
    autonomousChooser.AddOption(name, autoCommands.back().get());
  }

  // Default value and send to ShuffleBoard
  autonomousChooser.SetDefaultOption("no_auto", noAutoCommand.get());
  frc::SmartDashboard::PutData("Auto Chooser", &autonomousChooser);
}

void RobotContainer::setAutoDefaults() {
  // By default leave the drivetrain stopped so the robot doesnt move if
  // something goes wrong. No other defaults are given so the manuiulator and
  // claw retain thier state during auto.
  driveSubsystem.SetDefaultCommand(
      frc2::RunCommand([this]() { driveSubsystem.stop(); }, {&driveSubsystem}));
}

void RobotContainer::setTeleopDefaults() {
  // Default arcade drive
  driveSubsystem.SetDefaultCommand(
      driveSubsystem.arcadeDriveCommand(driveGamepad));

  // Default manual manipulator control
  manipulatorSubsystem.SetDefaultCommand(frc2::RunCommand([this]() { 
    // manipulatorSubsystem.setElevatorHeightPercent(joystick.GetThrottle());
    // manipulatorSubsystem.incArmAngle(1.5_deg * -joystick.GetX());
  }, {&manipulatorSubsystem}));

  // Default manual claw control
  clawSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        if (driveGamepad.GetRawButton(2)) {
          clawSubsystem.setClawClosed(false);
        } else {
          clawSubsystem.setClawClosed(true);
        }
      },
      {&clawSubsystem}));
}

void RobotContainer::startAutoCommand() {
  // Get command from chooser to scheduel
  currentAuto = autonomousChooser.GetSelected();
  currentAuto->Schedule();
}

void RobotContainer::endAutoCommand() {
  // Cancel selected auto
  currentAuto->Cancel();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  joystick.Button(7).OnTrue(
      manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::compactState));
  joystick.Button(8).OnTrue(manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubeHighState));
  joystick.Button(9).OnTrue(
      manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::cubeMidState));
  joystick.Button(10).OnTrue(manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubePickupState));
  joystick.Button(11).OnTrue(
      manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::coneMidState));
  joystick.Button(12).OnTrue(manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::conePickupState));
}
