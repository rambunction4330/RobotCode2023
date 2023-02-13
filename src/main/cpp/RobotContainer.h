// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <iostream>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/FunctionalCommand.h>

#include <pathplanner/lib/auto/RamseteAutoBuilder.h>

#include <rmb/controller/LogitechGamepad.h>
#include <rmb/controller/LogitechJoystick.h>

#include "drivetrain/commands/BalanceCommand.h"
#include "drivetrain/DriveSubsystem.h"
#include "manipulator/ManipulatorSubsystem.h"
#include "claw/ClawSubsystem.h"

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

  /**************
   * Subsystems *
   **************/
  rmb::LogitechJoystick joystick{1, 0.0, true};
  rmb::LogitechGamepad driveGamepad{0, 0.0, true};
  DriveSubsystem driveSubsystem;
  //ManipulatorSubsystem manipulatorSubsystem; 
  //ClawSubsystem clawSubsystem; 

  /****************
   * Path Planner *
   ****************/
  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap {
    // Drivetrain Commands
    {"drivetrain_balance", std::make_shared<BalanceCommand>(driveSubsystem)},

    // Manipulator Commands
    /*{"manipulator_compact", manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::compactState).Unwrap()},
    {"manipulator_high", manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::highState).Unwrap()},
    {"manipulator_mid", manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::midState).Unwrap()},
    {"manipulator_low_compact", manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::lowCompactState).Unwrap()},
    {"manipulator_low_reach", manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::lowReachState).Unwrap()},
 
    // Claw Commands
    {"claw_close", clawSubsystem.getClosedCommand(true).Unwrap()},
    {"claw_open", clawSubsystem.getClosedCommand(false).Unwrap()}*/
  };

  // Auto Builder
  pathplanner::RamseteAutoBuilder autoBuilder {
    [this]() { return driveSubsystem.getPose(); }, [this](frc::Pose2d initPose) { 
      driveSubsystem.resetOdometry(initPose); 
    },
    DriveConstants::ramseteController, DriveConstants::kinematics,
    [this](units::meters_per_second_t leftVelocity, units::meters_per_second_t rightVelocity) { 
      driveSubsystem.driveWheelSpeeds(leftVelocity, rightVelocity); 
    }, eventMap, {&driveSubsystem}, true
  };


  /****************
   * Auto Chooser *
   ****************/
  frc2::CommandPtr noAutoCommand = frc2::CommandPtr(frc2::PrintCommand("NO AUTO\n"));
  std::map<std::string, frc2::CommandPtr> autoCommands;
  frc2::Command* currentAuto = noAutoCommand.get();

  frc::SendableChooser<frc2::Command*> autonomousChooser;
};
