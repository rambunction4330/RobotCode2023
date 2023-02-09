// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <iostream>

#include <frc/Filesystem.h>
#include <frc/DriverStation.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/Trajectory.h>

#include <frc2/command/RunCommand.h>

#include <pathplanner/lib/PathPlanner.h>

#include "drivetrain/commands/BalanceCommand.h"

RobotContainer::RobotContainer() {
  // Set Default Commands
  driveSubsystem.SetDefaultCommand(driveSubsystem.arcadeDriveCommand(driveGamepad));

  manipulatorSubsystem.SetDefaultCommand(frc2::RunCommand([this]() { 
    manipulatorSubsystem.setElevatorHeightPercent(joystick.GetThrottle());
    manipulatorSubsystem.incArmPositon(0.5_deg * -joystick.GetX());
  }, {&manipulatorSubsystem}));

  clawSubsystem.SetDefaultCommand(frc2::RunCommand([this]() {
    if (joystick.GetTriggerPressed()) {
      clawSubsystem.toggleClaw();
    }
  }, {&clawSubsystem}));

  // Configure button bindings
  ConfigureBindings();

  // Setup Autonomouse Routines
  autoCommands.emplace("Leave Community Left", frc2::PrintCommand("NOT IMPLEMENTED\n"));
  autoCommands.emplace("Leave Community Right", frc2::PrintCommand("NOT IMPLEMENTED\n"));
  autoCommands.emplace("Put Low Leave Community Left", frc2::PrintCommand("NOT IMPLEMENTED\n"));
  autoCommands.emplace("Put Low Leave Community Right", frc2::PrintCommand("NOT IMPLEMENTED\n"));
  autoCommands.emplace("Put Mid Leave Community Left", frc2::PrintCommand("NOT IMPLEMENTED\n"));
  autoCommands.emplace("Put Mid Leave Community Right", frc2::PrintCommand("NOT IMPLEMENTED\n"));

  // Example Auto
  autoCommands.emplace("Cube High Balance", autoBuilder.fullAuto(pathplanner::PathPlanner::loadPathGroup("cube_high_balance", 1.75_mps, 2.0_mps_sq, true)));
    autoCommands.emplace("Balance Test", autoBuilder.fullAuto(pathplanner::PathPlanner::loadPathGroup("balance_test", 1.0_mps, 1.0_mps_sq, true)));

  autoCommands.emplace("Put Low Balance", frc2::PrintCommand("NOT IMPLEMENTED\n"));
  autoCommands.emplace("Put Mid Balance", frc2::PrintCommand("NOT IMPLEMENTED\n"));
  
  // Setup Auto Chooser
  autonomousChooser.SetDefaultOption("No Auto", noAutoCommand.get());
  for (auto& [key, value] : autoCommands) {
    autonomousChooser.AddOption(key, value.get());
  }

  frc::SmartDashboard::PutData("Auto Chooser", &autonomousChooser);
}

void RobotContainer::startAutoCommand() {
  currentAuto = autonomousChooser.GetSelected();
  currentAuto->Schedule();
}

void RobotContainer::endAutoCommand() {
  currentAuto->Cancel();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
}
