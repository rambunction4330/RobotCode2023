// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/commands/BalanceCommand.h"

#include <units/math.h>
#include <iostream>

BalanceCommand::BalanceCommand(DriveSubsystem& drive) : 
  driveSubsystem(drive) {

  // Reqires drive subsystem. 
  AddRequirements({&driveSubsystem});
}

// Called when the command is initially scheduled.
void BalanceCommand::Initialize() {

  // Resets corrdinate system of command without effecting the rest of the 
  // robot.
  offset = driveSubsystem.getPose();
}

// Called repeatedly when this Command is scheduled to run
void BalanceCommand::Execute() {
  // Robot pitch with angle correction
  double feedback = -driveSubsystem.getRobotPitch().to<double>() - 0.06;

  // Clamp small angles to zero
  if (abs(feedback) < 0.005) { feedback = 0.0; }

  // Calculate positon from PID and clamp to limits
  units::meter_t goal = units::meter_t(balanceController.Calculate(feedback));
  goal = std::clamp(goal, minX, maxX);
  
  // Genrates profile to achive position
  frc::TrapezoidProfile<units::meters> profile = {
    constraints, {goal, 0.0_mps}, 
    {(driveSubsystem.getPose() - offset).X(), 0.0_mps}
  };

  // Drive to position from profile. Typically we would sample at 20 ms since 
  // that is the loop period. Instead it jumps to 40_ms to spee up balancing. 
  driveSubsystem.driveChassisSpeeds({
    profile.Calculate(40_ms).velocity, 0.0_mps, 0.0_rpm
  });

  // Conditionaly start a timer if the robot is balanced. If not the timer is 
  // reset. After 2 seconds of being balanced teh command will end.
  if (units::math::abs(driveSubsystem.getRobotPitch()) < 2.5_deg) {
    balanceTimer.Start();
  } else {
    balanceTimer.Stop();
    balanceTimer.Reset();
  }
}

// Called once the command ends or is interrupted.
void BalanceCommand::End(bool interrupted) {
  // Sop the drivetrain once the robot balances.
  driveSubsystem.stop(); 
}

// Returns true when the command should end.
bool BalanceCommand::IsFinished() {
  // Aftering being level for 2.0 seconds, end the command.
  return balanceTimer.Get() > 2.0_s;
}
