// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <array>
#include <iostream>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include <pathplanner/lib/auto/RamseteAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#include <rmb/controller/LogitechGamepad.h>
#include <rmb/controller/LogitechJoystick.h>

#include "drivetrain/commands/BalanceCommand.h"
#include "drivetrain/DriveSubsystem.h"
#include "manipulator/ManipulatorSubsystem.h"
#include "claw/ClawSubsystem.h"
#include "units/acceleration.h"
#include "units/velocity.h"

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

  void setAutoDefaults();
  void setTeleopDefaults();

  void startAutoCommand();
  void endAutoCommand();

  void buildAutos() {
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

 private: 
  void ConfigureBindings(); 

  /**************
   * Subsystems *
   **************/
  frc2::CommandXboxController driveGamepad{0};
  frc2::CommandPS4Controller manipulatorGamepad{1};

  // rmb::LogitechJoystick driveStick{0, 0.0, true};
  DriveSubsystem driveSubsystem;
  ManipulatorSubsystem manipulatorSubsystem; 
  ClawSubsystem clawSubsystem; 

  /****************
   * Path Planner *
   ****************/

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap {
    // Drivetrain Commands
    {"drivetrain_balance", std::make_shared<BalanceCommand>(driveSubsystem)},

    //Manipulator Commands
    {"manipulator_compact", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::compactState
    ).Unwrap()},

    {"manipulator_cube_high", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubeHighState
    ).Unwrap()},

    {"manipulator_cube_mid", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubeMidState
    ).Unwrap()},

    {"manipulator_cube_pickup", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::cubePickupState
    ).Unwrap()},

    {"manipulator_cone_mid", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::coneMidState
    ).Unwrap()},

    {"manipulator_cone_pickup", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::conePickupState
    ).Unwrap()},

    {"manipulator_substation", manipulatorSubsystem.getStateCommand(
      ManipulatorSubsystem::substationState
    ).Unwrap()},
 
    // Claw Commands
    {"claw_close", clawSubsystem.getClosedCommand(true).Unwrap()},
    {"claw_open", clawSubsystem.getClosedCommand(false).Unwrap()},
  };

  // Auto Builder
  pathplanner::RamseteAutoBuilder autoBuilder {
    [this]() { return driveSubsystem.getPose(); }, 
    [this](frc::Pose2d initPose) { 
      driveSubsystem.resetOdometry(initPose); 
    }, DriveConstants::ramseteController, DriveConstants::kinematics,
    [this](units::meters_per_second_t left, units::meters_per_second_t right) { 
      driveSubsystem.driveWheelSpeeds(left, right); 
    }, eventMap, {&driveSubsystem}, true
  };


  /****************
   * Auto Chooser *
   ****************/
  struct AutoConfiguration {
    std::string name;
    units::meters_per_second_t maxVelocity = 2.0_mps;
    units::meters_per_second_squared_t maxAcceleration = 2.0_mps_sq;
    bool reversed = true;
  };

  std::array<AutoConfiguration, 10> autoNames {
    {
      {"wall_move"},
      {"wall_balance"},
      {"wall_put"},
      {"center_move_wall"},
      {"center_move_sub"},
      {"center_balance", 1.0_mps, 1.0_mps_sq},
      {"sub_move"},
      {"sub_balance"},
      {"sub_put", 1.5_mps, 1.5_mps_sq},
      {"test", 1.5_mps, 1.5_mps_sq},
    }
  };

  frc2::CommandPtr noAutoCommand = frc2::PrintCommand("NO AUTO\n").ToPtr();
  std::vector<frc2::CommandPtr> autoCommands;
  frc2::Command* currentAuto = noAutoCommand.get();

  frc::SendableChooser<frc2::Command*> autonomousChooser;
};
