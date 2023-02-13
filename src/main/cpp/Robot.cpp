#include "Robot.h"

#include <cameraserver/CameraServer.h>

#include <iostream>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc2/command/PrintCommand.h>
#include <units/velocity.h>

void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::TeleopInit() {
  container.startTeleop();
}
void Robot::TeleopPeriodic()  {} 
void Robot::TeleopExit() {
  container.endTeleop();
}

void Robot::AutonomousInit() {
  container.startAutoCommand();
}

void Robot::AutonomousPeriodic()  {} 

void Robot::AutonomousExit()  {
  container.endAutoCommand();
} 

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::TestExit() {}
