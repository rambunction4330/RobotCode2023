// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <pathplanner/lib/auto/RamseteAutoBuilder.h>

#include "drivetrain/DriveSubsystem.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/PrintCommand.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class JTurnCommand : 
public frc2::CommandHelper<frc2::CommandBase, JTurnCommand> {
 public:

   JTurnCommand(DriveSubsystem& driveSubsystem, pathplanner::RamseteAutoBuilder& autoBuilder);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem& driveSubsystem;
  pathplanner::RamseteAutoBuilder& autoBuilder;
  std::unique_ptr<frc2::Command> trajectoryCommand = frc2::PrintCommand("ERROR\n").ToPtr().Unwrap();
};