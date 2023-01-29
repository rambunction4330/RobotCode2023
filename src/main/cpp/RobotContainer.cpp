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

  // Setup Auto Chooser
  autonomousChooser.SetDefaultOption("None", "None");
  autonomousChooser.AddOption("Leave_Left", "Leave_Left");
  autonomousChooser.AddOption("Leave_Right", "Leave_Right");
  autonomousChooser.AddOption("Place_Low_Leave_Left", "Place_Low_Leave_Left");
  autonomousChooser.AddOption("Place_Low_Leave_Right", "Place_Low_Leave_Right");
  autonomousChooser.AddOption("Place_Mid_Leave_Left", "Place_Mid_Leave_Left");
  autonomousChooser.AddOption("Place_Mid_Leave_Right", "Place_Mid_Leave_Right");
  autonomousChooser.AddOption("Balance", "Balance");
  autonomousChooser.AddOption("Place_Low_Balance", "Place_Low_Balance");
  autonomousChooser.AddOption("Place_Mid_Balance", "Place_Mid_Balance");
}

void RobotContainer::startAutoCommand() {
  // Grab alliance to know which side of feild we're on
  std::string alliance;
  switch (frc::DriverStation::GetAlliance()) {
  case frc::DriverStation::Alliance::kBlue:
    alliance = "blue";
    break;
  case frc::DriverStation::Alliance::kRed:
    alliance = "red";
    break;
  case frc::DriverStation::Alliance::kInvalid:
    return;
  }

  // Get and cycle through all the possible autos
  std::string selected = autonomousChooser.GetSelected();

  std::string trajectoryPath = frc::filesystem::GetDeployDirectory() 
                               + "/" + alliance
                               + "/" + selected + ".json";

  if (selected == "Leave_Left") {
    frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(trajectoryPath);
    autoCommand = driveSubsystem.getTrajectoryCommand(trajectory);
  } else if (selected == "Leave_Right") {
    frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(trajectoryPath);
    autoCommand = driveSubsystem.getTrajectoryCommand(trajectory);
  } else if (selected == "Place_Low_Leave_Left") {
    // TODO: Impliment "Place_Low_Leave_Left" Auto
  } else if (selected == "Place_Low_Leave_Right") {
    // TODO: Impliment "Place_Low_Leave_Right" Auto
  } else if (selected == "Place_Mid_Leave_Left") {
    // TODO: Impliment "Place_Mid_Leave_Left" Auto
  } else if (selected == "Place_Mid_Leave_Right") {
    // TODO: Impliment "Place_Mid_Leave_Right" Auto
  } else if (selected == "Balance") {
    frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(trajectoryPath);
    autoCommand = driveSubsystem.getTrajectoryCommand(trajectory).AndThen(frc2::CommandPtr(BalanceCommand(driveSubsystem)));
  } else if (selected == "Place_Low_Balance") { 
    // TODO: Impliment "Place_Low_Balance" Auto
  } else if (selected == "Place_Mid_Balance") {
    // TODO: Impliment "Place_Mid_Balance" Auto
  }

  autoCommand.Schedule();
}

void RobotContainer::endAutoCommand() {
  autoCommand.Cancel();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
}
