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
  autoCommands["Leave Community Left"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };

  autoCommands["Leave Community Right"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };

  autoCommands["Put Low Leave Community Left"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };

  autoCommands["Put Low Leave Community Right"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };

  autoCommands["Put Mid Leave Community Left"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };

  autoCommands["Put Mid Leave Community Right"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };
  autoCommands["Balance"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };

  autoCommands["Put Low Balance"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };

  autoCommands["Put Mid Balance"] = {
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: RED\n")), 
    frc2::CommandPtr(frc2::PrintCommand("NOT IMPLEMENTED: BLUE\n"))
  };
  
  // Setup Auto Chooser
  autonomousChooser.SetDefaultOption("No Auto", noAutoCommand);
  for (const auto& [key, value] : autoCommands) {
    autonomousChooser.AddOption(key, value);
  }
}

void RobotContainer::startAutoCommand() {
  auto selected = autonomousChooser.GetSelected();
  switch (frc::DriverStation::GetAlliance()) {
    case frc::DriverStation::Alliance::kRed:
      currentAuto = std::get<0>(selected)
      currentAuto.Schedule();
      break;
    case frc::DriverStation::Alliance::kBlue:
      currentAuto = std::get<1>(selected)
      currentAuto.Schedule();
      break;
    default:
      return;
  }
}

void RobotContainer::endAutoCommand() {
  currentAuto.Cancel();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
}
