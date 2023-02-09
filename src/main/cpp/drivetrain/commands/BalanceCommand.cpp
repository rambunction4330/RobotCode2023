// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/commands/BalanceCommand.h"

#include <units/math.h>
#include <iostream>

BalanceCommand::BalanceCommand(DriveSubsystem& drive) : 
  driveSubsystem(drive) {
  AddRequirements({&driveSubsystem});
}

// Called when the command is initially scheduled.
void BalanceCommand::Initialize() {

  // Resets corrdinate system of command without effecting the rest of the robot.
  offset = driveSubsystem.getPose();
}

// Called repeatedly when this Command is scheduled to run
void BalanceCommand::Execute() {
  // Increments goal based on gain robot pitch
  double feedback = -driveSubsystem.getRobotPitch().to<double>() - 0.06;
  if (abs(feedback) < 0.005) { feedback = 0.0; }
  goal = units::meter_t(balanceController.Calculate(feedback));
  goal = std::clamp(goal, minX, maxX);
  
  // Genrates profile
  frc::TrapezoidProfile<units::meters> profile = {constraints, 
                                                  {goal, 0.0_mps}, 
                                                  {(driveSubsystem.getPose() - offset).X(), 0.0_mps}};

  // Drives profile
  driveSubsystem.driveChassisSpeeds({profile.Calculate(40_ms).velocity, 0.0_mps, 0.0_rpm});

  // Start timer if level and reset of not
  if (units::math::abs(driveSubsystem.getRobotPitch()) < 2.5_deg) {
    balanceTimer.Start();
  } else {
    balanceTimer.Stop();
    balanceTimer.Reset();
  }
}

// Called once the command ends or is interrupted.
void BalanceCommand::End(bool interrupted) {
  driveSubsystem.stop(); 
}

// Returns true when the command should end.
bool BalanceCommand::IsFinished() {
  // Aftering being level for 2.0 seconds the command ends
  return balanceTimer.Get() > 2.0_s;
}
