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
#include <frc/GenericHID.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/shuffleboard/BuiltInWidgets.h>

#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/ScheduleCommand.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathConstraints.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPoint.h>
#include <pathplanner/lib/auto/RamseteAutoBuilder.h>

#include "drivetrain/DriveSubsystem.h"
#include "drivetrain/commands/BalanceCommand.h"
#include "drivetrain/commands/JTurnCommand.h"
#include "autoalignment/Autoalignment.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/InstantCommand.h"
#include "manipulator/ManipulatorSubsystem.h"
#include "claw/ClawConstants.h"

RobotContainer::RobotContainer() {
  // Configure button bindings
  ConfigureBindings();

  // Setup Auto Routines
  for (auto autoConfig : autoNames) {
    
    std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup = 
      pathplanner::PathPlanner::loadPathGroup(
        autoConfig.name, autoConfig.maxVelocity, 
        autoConfig.maxAcceleration, autoConfig.reversed);

    std::vector<pathplanner::PathPlannerTrajectory> newTrajectoryGroup;
    newTrajectoryGroup.reserve(trajectoryGroup.size());

    for (auto trajectory: trajectoryGroup) {
      newTrajectoryGroup.push_back(
        pathplanner::PathPlannerTrajectory::transformTrajectoryForAlliance(
          trajectory, frc::DriverStation::GetAlliance())
      );
    }

    // Build Commands
    autoCommands.emplace_back(
      driveSubsystem.getAutoCommand(newTrajectoryGroup, eventMap)
    );

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
  manipulatorSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        // Dead zones
        double leftY = std::abs(manipulatorGamepad.GetRawAxis(1)) < 0.05
                           ? 0.0
                           : manipulatorGamepad.GetRawAxis(1);
        manipulatorSubsystem.incArmAngle(0.5_deg * -leftY);

        // Dead zones
        double rightY = std::abs(manipulatorGamepad.GetRawAxis(5)) < 0.05
                            ? 0.0
                            : manipulatorGamepad.GetRawAxis(5);
        manipulatorSubsystem.setElevatorHeight(
            manipulatorSubsystem.getTargetElevatorHeight() +
            (1.0_in * -rightY));
      },
      {&manipulatorSubsystem}));

  // Default manual claw control
  clawSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        using namespace ClawSubsystemConstants;
        // Leave rumble off by default.
        driveGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
        manipulatorGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);

        // Open claw when Padddles is pressed.
        if (driveGamepad.GetRawButton(1)) {
          clawSubsystem.setClawClosed(false);

          // Rumble while stalling motor to open the claw.
          driveGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
          manipulatorGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.75); 
        } else {
          // Set boost to help claw depending on button presses.
          double closeBoost = 0.0;

          // Calculate needed boost
          auto currentAcceleration = driveSubsystem.getAcceleration();
           if (driveGamepad.GetRightBumper()) {
            closeBoost = (closeBoostWithoutRamp + closeBoostMax) / 2.0;

            // Ruble when applying extra closing boost.
            driveGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
             manipulatorGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.4); 
          } else if (driveGamepad.GetLeftBumper()) {
            closeBoost = closeBoostMax;

            // Ruble when applying extra closing boost.
            driveGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
             manipulatorGamepad.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5); 
          } else if (currentAcceleration > closeAssistRampMinAcceleration) {
            closeBoost =
                (closeBoostMax - closeBoostWithoutRamp) /
                    (closeAssistRampMaxAcceleration -
                     closeAssistRampMinAcceleration) *
                    (currentAcceleration - closeAssistRampMinAcceleration) + 
                closeBoostWithoutRamp;
            
            closeBoost = boostFilter.Calculate(closeBoost);

            closeBoost = std::min(closeBoost, closeBoostMax);
          } else {
            closeBoost = closeBoostWithoutRamp;
          }

          clawSubsystem.setClawClosed(true, closeBoost);
        }
        frc::SmartDashboard::PutBoolean("Claw Closed", clawSubsystem.getClawClosed());
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
  // Manipulator Button Bindings.
  manipulatorGamepad.R1().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::substationState));
  manipulatorGamepad.L1().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::coneHighState));
  manipulatorGamepad.Triangle().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::cubePickupState));
  manipulatorGamepad.Circle().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::compactState));
  manipulatorGamepad.Cross().OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::conePickupState));
  frc2::Trigger([this]() { return manipulatorGamepad.GetPOV() == 0; }).OnTrue(manipulatorSubsystem.getStateCommand(ManipulatorSubsystem::coneMidState));
  manipulatorGamepad.R2().WhileTrue(manipulatorSubsystem.manualZeroArmCommand(manipulatorGamepad));

  // Drive Button Bindings
  /*driveGamepad.B().WhileTrue(autoalignment::AutoAlignmentCommand(driveSubsystem, autoBuilder).ToPtr());*/
}
