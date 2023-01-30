// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <pair>

#include <frc/smartdashboard/SendableChooser.h>

#include <rmb/controller/LogitechGamepad.h>

#include "drivetrain/DriveSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  void startAutoCommand();
  void endAutoCommand();

 private: 
  void ConfigureBindings();

  rmb::LogitechGamepad driveGamepad{0, 0.1, true};
  DriveSubsystem driveSubsystem;

  std::pair<frc2::CommandPtr, frc2::CommandPtr> noAutoCommand = {
    frc2::CommandPtr(frc2::PrintCommand("\"NO AUTO; RED\"\n")),
    frc2::CommandPtr(frc2::PrintCommand("\"NO AUTO; BLUE\"\n"))
  }

  std::map<std::string, std::pair<frc2::CommandPtr, frc2::CommandPtr>> autoCommands;
  frc::SendableChooser<std::pair<const frc2::CommandPtr&, const frc2::CommandPtr&>> autonomousChooser;

  frc2::CommandPtr& currentAuto = noAutoCommand;
};
