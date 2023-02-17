// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>

#include "drivetrain/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class BalanceCommand : public frc2::CommandHelper<frc2::CommandBase, BalanceCommand> {
 public:
  using Gain = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using Gain_t = units::unit_t<Gain>;

  BalanceCommand(DriveSubsystem& driveSubsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  DriveSubsystem& driveSubsystem;
  frc::Pose2d offset;

  frc::TrapezoidProfile<units::meters>::Constraints constraints = {1.0_mps, 7.0_mps_sq};
  frc::PIDController balanceController {0.12, 0.175, 0.0};
  units::meter_t goal = 0.0_m;
  units::meter_t minX = -0.4_m;
  units::meter_t maxX = 0.4_m;

  frc::Timer balanceTimer;
};
