// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <tuple>
#include <iostream>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/FunctionalCommand.h>

#include <pathplanner/lib/auto/RamseteAutoBuilder.h>

#include <rmb/controller/LogitechGamepad.h>
#include <rmb/controller/LogitechJoystick.h>

#include "drivetrain/commands/BalanceCommand.h"
#include "drivetrain/DriveSubsystem.h"
#include "manipulator/ManipulatorSubsystem.h"

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

  rmb::LogitechJoystick joystick{1, 0.05, true};
  rmb::LogitechGamepad driveGamepad{0, 0.05, true};
  DriveSubsystem driveSubsystem;
  ManipulatorSubsystem manipulatorSubystem; 

  frc2::CommandPtr noAutoCommand = frc2::CommandPtr(frc2::PrintCommand("NO AUTO\n"));
  std::map<std::string, frc2::CommandPtr> autoCommands;
  frc::SendableChooser<frc2::Command*> autonomousChooser;

  frc2::Command* currentAuto = noAutoCommand.get();

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap {
    {"lower", std::make_shared<frc2::FunctionalCommand>(frc2::FunctionalCommand(
      [this]() {},
      [this]() {
        manipulatorSubystem.setElevatorHeight(38.0_in);
        manipulatorSubystem.setArmPosition(-82.0_deg);
        manipulatorSubystem.setClawClosed(false);
      },
      [this](bool) {},
      [this]() {
        return manipulatorSubystem.elevatorAtHeight() && manipulatorSubystem.armAtPosition();
      },
      {&manipulatorSubystem}
    ))},

    {"grab", std::make_shared<frc2::FunctionalCommand>(frc2::FunctionalCommand(
      [this]() {},
      [this]() {
        manipulatorSubystem.setClawClosed(true);
      },
      [this](bool) {},
      [this]() {
        return manipulatorSubystem.getClawClosed();
      },
      {&manipulatorSubystem}
    ))},

    {"pickup", std::make_shared<frc2::FunctionalCommand>(frc2::FunctionalCommand(
      [this]() {},
      [this]() {
        manipulatorSubystem.setElevatorHeight(0.0_in);
        manipulatorSubystem.setArmPosition(90.0_deg);
        manipulatorSubystem.setClawClosed(true);
      },
      [this](bool) {},
      [this]() {
        return manipulatorSubystem.elevatorAtHeight() && manipulatorSubystem.armAtPosition();
      },
      {&manipulatorSubystem}
    ))}
  };

  pathplanner::RamseteAutoBuilder autoBuilder {
    [this]() { return driveSubsystem.getPose(); }, [this](frc::Pose2d initPose) { 
      driveSubsystem.resetOdometry(initPose); 
    },
    DriveConstants::ramseteController, DriveConstants::kinematics,
    [this](units::meters_per_second_t leftVelocity, units::meters_per_second_t rightVelocity) { 
      driveSubsystem.driveWheelSpeeds(leftVelocity, rightVelocity); 
    }, eventMap, {&driveSubsystem}, true
  };
};
