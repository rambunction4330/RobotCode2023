// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <array>
#include <iostream>

#include <units/acceleration.h>
#include <units/velocity.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include <pathplanner/lib/auto/RamseteAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#include <rmb/controller/LogitechGamepad.h>
#include <rmb/controller/LogitechJoystick.h>

#include "drivetrain/DriveSubsystem.h"
#include "drivetrain/commands/BalanceCommand.h"
#include "drivetrain/commands/JTurnCommand.h"
#include "frc/filter/LinearFilter.h"
#include "manipulator/ManipulatorSubsystem.h"
#include "claw/ClawSubsystem.h"

#define FEILD_WIDTH 16.53_m

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

 private: 
  void ConfigureBindings(); 

  /***************
   * Controllers *
   ***************/
  frc2::CommandXboxController driveGamepad{0};
  frc2::CommandPS4Controller manipulatorGamepad{1};

  /**************
   * Subsystems *
   **************/
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
    {"claw_close_boost", clawSubsystem.getClosedCommand(true, 0.15).Unwrap()},
    {"claw_open", clawSubsystem.getClosedCommand(false).Unwrap()},

    {"pause", frc2::WaitCommand(0.7_s).ToPtr().Unwrap()},
  };

  // Auto Builder
  pathplanner::RamseteAutoBuilder autoBuilder {
    [this]() { 
      // Absolute piositon of robt onthe feils with the right corrner of the 
      // blue alliance side as the origin.
      frc::Pose2d rawPose = driveSubsystem.getPose();
      
      // Autos are designed for the blue alliance side, so on that side the 
      // raw feild position can be used. When starting on the red side, the 
      // provided must be modified to trick the path following command into 
      // thinking it is still on the red side.
      if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
        return rawPose;
      }

      // Mirros field position from red back to blue side
      frc::Translation2d translation {FEILD_WIDTH - rawPose.X(), rawPose.Y()};

      // Flip rotation to be facing the other direction.
      frc::Rotation2d rotation = -rawPose.Rotation() + frc::Rotation2d(180_deg);

      // return transformed position.
      return frc::Pose2d(translation, rotation);
    }, 
    [this](frc::Pose2d initPose) { 
      // Autos are designed for the blue alliance side, so on that side the 
      // absolute feild position can be used. When starting on the red side,
      // the provided must be modified to trick the path following command into 
      // thinking it is still on the red side.
      if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
        driveSubsystem.resetOdometry(initPose);
        return;
      }

      // Mirror the start position over tot he red side of th field.
      frc::Translation2d translation {FEILD_WIDTH - initPose.X(), initPose.Y()};

      // Flip the rotation to face the oposite direction.
      frc::Rotation2d rotation = -initPose.Rotation() + frc::Rotation2d(180_deg);

      // Reset to transformed position.
      driveSubsystem.resetOdometry(frc::Pose2d(translation, rotation)); 
    }, DriveConstants::ramseteController, DriveConstants::kinematics,
    [this](units::meters_per_second_t left, units::meters_per_second_t right) {
      if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) { 
        driveSubsystem.driveWheelSpeeds(left, right); 
      } else {
        driveSubsystem.driveWheelSpeeds(right, left); 
      }
    }, eventMap, {&driveSubsystem}, false
  };

  // Struter to keep track of auto file and relevant configs.
  struct AutoConfiguration {
    std::string name;
    units::meters_per_second_t maxVelocity = 2.0_mps;
    units::meters_per_second_squared_t maxAcceleration = 2.0_mps_sq;
    bool reversed = true;
  };

  // List of possible autos and relevant configs.
  std::vector<AutoConfiguration> autoNames {
    {
      {"wall_move_cube"},
      {"wall_move_cone"},
      {"wall_balance_cube"},
      {"wall_balance_cone"},
      {"center_move_wall_cube"},
      {"center_move_wall_cone"},
      {"center_move_sub_cube"},
      {"center_move_sub_cone"},
      {"center_balance_cube", 1.25_mps, 0.75_mps_sq},
      {"center_balance_cone", 1.25_mps, 0.75_mps_sq},
      {"sub_move_cube"},
      {"sub_move_cone"},
      {"sub_balance_cube"},
      {"sub_balance_cone"},
      {"sub_put_cone", 3.0_mps, 2.8_mps_sq},
      {"center_stay_cube", 0.1_mps, 0.001_mps_sq}
    }
  };

  /****************
   * Auto Chooser *
   ****************/
  frc2::CommandPtr noAutoCommand = frc2::PrintCommand("NO AUTO\n").ToPtr();
  std::vector<frc2::CommandPtr> autoCommands;
  frc2::Command* currentAuto = noAutoCommand.get();

  frc::LinearFilter<double> boostFilter = frc::LinearFilter<double>::MovingAverage(50);

  frc::SendableChooser<frc2::Command*> autonomousChooser;
};
