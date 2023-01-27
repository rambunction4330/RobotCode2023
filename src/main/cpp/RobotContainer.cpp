// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <iostream>

#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/Trajectory.h>

#include "drivetrain/commands/BalanceCommand.h"

RobotContainer::RobotContainer() {
  // Set Default Commands
  // driveSubsystem.SetDefaultCommand(driveSubsystem.arcadeDriveCommand(driveGamepad));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::scheduleAutoCommand() {
  // if (autoCommand.IsScheduled()) { return; }
  
  // // Zero dometry
  // driveSubsystem.resetOdometry(frc::Pose2d(0.0_m, 0.0_m, 0.0_rad));
  // if (!autoCommand.IsScheduled()) {
  //   frc::TrajectoryConfig config{0.75_mps, 2.0_mps_sq};
  //   config.SetKinematics(DriveConstants::kinematics);

  //   frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //     frc::Pose2d(0.0_m, 0.0_m, 0.0_rad), 
  //     {{1.2_m, 0.0_m}}, 
  //     frc::Pose2d(1.5_m, -1.4_m, 180.0_deg),
  //     config
  //   );

  //   // Drives a trajectory to the front of the charging pad and  the balances on it
  //   autoCommand = driveSubsystem.getTrajectoryCommand(trajectory).AndThen(frc2::CommandPtr(BalanceCommand(driveSubsystem)));

  //   // Start to command
  //   autoCommand.Schedule();
  // }
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
}
