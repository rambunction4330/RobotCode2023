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
#include "frc/GenericHID.h"
#include "frc2/command/button/Trigger.h"

RobotContainer::RobotContainer() {
  // Configure button bindings
  ConfigureBindings();

  // Setup Auto Routines
  for (auto autoConfig : autoNames) {
    // Build Command
    autoCommands.emplace_back(
        autoBuilder.fullAuto(pathplanner::PathPlanner::loadPathGroup(
            autoConfig.name, autoConfig.maxVelocity, autoConfig.maxAcceleration, autoConfig.reversed)));

    // Add to chooser
    autonomousChooser.AddOption(autoConfig.name, autoCommands.back().get());
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
    double leftY = std::abs(manipulatorGamepad.GetRawAxis(1)) < 0.05 ? 0.0 : manipulatorGamepad.GetRawAxis(1);
    manipulatorSubsystem.incArmAngle(1.0_deg * -leftY);

    double rightY = std::abs(manipulatorGamepad.GetRawAxis(5)) < 0.05 ? 0.0 : manipulatorGamepad.GetRawAxis(5);
    manipulatorSubsystem.setElevatorHeight(manipulatorSubsystem.getTargetElevatorHeight() + (1.5_in * -rightY));
  }, {&manipulatorSubsystem}));

  // Default manual claw control
  clawSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        double closeBoost = 0.0;
        driveGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0); 
        if (driveGamepad.GetRightBumper()) { closeBoost = 0.3; }
        if (driveGamepad.GetLeftBumper()) { 
          closeBoost = 0.6;
          driveGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0); 
        }

        if (driveGamepad.GetRawButton(2)) {
          clawSubsystem.setClawClosed(false);
          driveGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
        } else {
          clawSubsystem.setClawClosed(true, closeBoost);
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
  /*joystick.Button(7).OnTrue(
      manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::compactState));
  joystick.Button(8).OnTrue(manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubeHighState));
  joystick.Button(9).OnTrue(
      manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::coneHighState));
  joystick.Button(10).OnTrue(manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubePickupState));
  joystick.Button(11).OnTrue(
      manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::coneMidState));
  joystick.Button(12).OnTrue(manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::conePickupState));*/

  manipulatorGamepad.R1().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::substationState));
  manipulatorGamepad.L1().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::coneHighState));
  manipulatorGamepad.Triangle().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::cubePickupState));
  manipulatorGamepad.Circle().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::compactState));
  manipulatorGamepad.Cross().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::conePickupState));
  frc2::Trigger([this]() { return manipulatorGamepad.GetPOV() == 0; }).OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::coneMidState));
}
