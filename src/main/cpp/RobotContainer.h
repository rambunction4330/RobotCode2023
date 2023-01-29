// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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


  frc::SendableChooser<std::string> autonomousChooser;

  // Holds the autonomus command thats currently being used because the 
  // for some reason the command schedueler refuses to take ownership
  // of it.
  frc2::CommandPtr autoCommand{frc2::PrintCommand("THERE'S NO AUTO")};
};
