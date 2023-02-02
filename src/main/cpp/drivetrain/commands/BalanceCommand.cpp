// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/commands/BalanceCommand.h"

#include <units/math.h>
#include <iostream>

BalanceCommand::BalanceCommand(DriveSubsystem& drive, bool r) : 
  driveSubsystem(drive) {
  AddRequirements({&driveSubsystem});

  // Sets min, max and inital goal besed on reverse state
  goal *= r ? -1 : 1;
  minX *= r ? -1 : 1;
  minX *= r ? -1 : 1;
  gain *= r ? -1 : 1;
}

// Called when the command is initially scheduled.
void BalanceCommand::Initialize() {

  // Resets corrdinate system of command without effecting the rest of the robot.
  offset = driveSubsystem.getPose();
}

// Called repeatedly when this Command is scheduled to run
void BalanceCommand::Execute() {
  // Increments goal based on gain robot pitch
  goal += (driveSubsystem.getRobotPitch() * gain);
  goal = std::clamp(goal, minX, maxX);
  
  // Genrates profile
  frc::TrapezoidProfile<units::meters> profile = {constraints, 
                                                  {goal, 0.0_mps}, 
                                                  {(driveSubsystem.getPose() - offset).X(), driveSubsystem.getChassisSpeeds().vx}};

  // Drives profile
  driveSubsystem.driveChassisSpeeds({profile.Calculate(20_ms).velocity, 0.0_mps, 0.0_rpm});

  // Start timer if level and reset of not
  if (units::math::abs(driveSubsystem.getRobotPitch()) < 2.5_deg) {
    balanceTimer.Start();
  } else {
    balanceTimer.Stop();
    balanceTimer.Reset();
  }

  std::cout << units::length::to_string((driveSubsystem.getPose() - offset).X()) << std::endl;
}

// Called once the command ends or is interrupted.
void BalanceCommand::End(bool interrupted) {
  driveSubsystem.stop(); 
}

// Returns true when the command should end.
bool BalanceCommand::IsFinished() {
  // Aftering being level for 2.0 seconds the command ends
  // return balanceTimer.Get() > 2.0_s;
  return false;
}
