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

#include "drivetrain/commands/BalanceCommand.h"

RobotContainer::RobotContainer() {
  // Set Default Commands
  driveSubsystem.SetDefaultCommand(driveSubsystem.arcadeDriveCommand(driveGamepad));

  // Configure button bindings
  ConfigureBindings();

  // Setup Autonomouse Routines
  autoCommands.emplace("Leave Community Left", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));

  autoCommands.emplace("Leave Community Right", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));

  autoCommands.emplace("Put Low Leave Community Left", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));

  autoCommands.emplace("Put Low Leave Community Right", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));

  autoCommands.emplace("Put Mid Leave Community Left", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));

  autoCommands.emplace("Put Mid Leave Community Right", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));

  autoCommands.emplace("Balance", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));

  autoCommands.emplace("Put Low Balance", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));

  autoCommands.emplace("Put Mid Balance", std::make_pair(
    frc2::PrintCommand("NOT IMPLEMENTED: RED\n"), 
    frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n")
  ));
  
  // Setup Auto Chooser
  autonomousChooser.SetDefaultOption("No Auto", {noAutoCommand.first.get(), noAutoCommand.second.get()});
  for (auto& [key, value] : autoCommands) {
    autonomousChooser.AddOption(key, {value.first.get(), value.second.get()});
  }
}

void RobotContainer::startAutoCommand() {
  auto selected = autonomousChooser.GetSelected();
  switch (frc::DriverStation::GetAlliance()) {
    case frc::DriverStation::Alliance::kRed:
      currentAuto = selected.first;
      currentAuto->Schedule();
      break;
    case frc::DriverStation::Alliance::kBlue:
      currentAuto = selected.second;
      currentAuto->Schedule();
      break;
    default:
      return;
  }
}

void RobotContainer::endAutoCommand() {
  currentAuto->Cancel();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
}
