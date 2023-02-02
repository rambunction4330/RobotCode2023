// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <unordered_map>
#include <tuple>

#include <frc/smartdashboard/SendableChooser.h>

#include <pathplanner/lib/auto/RamseteAutoBuilder.h>

#include <rmb/controller/LogitechGamepad.h>

#include "drivetrain/DriveSubsystem.h"
#include "drivetrain/commands/BalanceCommand.h"

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

  frc2::CommandPtr noAutoCommand = frc2::CommandPtr(frc2::PrintCommand("NO AUTO\n"));
  std::map<std::string, frc2::CommandPtr> autoCommands;
  frc::SendableChooser<frc2::Command*> autonomousChooser;

  frc2::Command* currentAuto = noAutoCommand.get();

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap {
    {"balance", std::make_shared<BalanceCommand>(driveSubsystem, false)}
  };

  pathplanner::RamseteAutoBuilder autoBuilder {
    [this]() { return driveSubsystem.getPose(); }, [this](frc::Pose2d initPose) { driveSubsystem.resetOdometry(initPose); },
    DriveConstants::ramseteController, DriveConstants::kinematics,
    [this](units::meters_per_second_t leftVelocity, units::meters_per_second_t rightVelocity) { 
      driveSubsystem.driveWheelSpeeds(leftVelocity, rightVelocity); 
    }, eventMap, {&driveSubsystem}, true
  };
};
