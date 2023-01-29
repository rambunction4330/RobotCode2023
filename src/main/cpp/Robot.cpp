#include "Robot.h"

#include <iostream>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc2/command/PrintCommand.h>
#include <units/velocity.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()  {} 
void Robot::TeleopExit() {}

void Robot::AutonomousInit() {
  container.scheduleAutoCommand();
}
void Robot::AutonomousPeriodic()  {
  // container.driveSubsystem.driveWheelSpeeds(0.75_mps, 0.75_mps);
} 
void Robot::AutonomousExit()  {} 

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::TestExit() {}