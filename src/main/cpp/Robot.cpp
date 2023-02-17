#include "Robot.h"

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::TeleopInit() {
  // Set proper default commands
  container.setTeleopDefaults();
}
void Robot::TeleopPeriodic()  {} 
void Robot::TeleopExit() {}

void Robot::AutonomousInit() {
  // Set proper default commands
  container.setAutoDefaults();

  // Start auto
  container.startAutoCommand();
}

void Robot::AutonomousPeriodic()  {} 

void Robot::AutonomousExit() {
  // Stop auto
  container.endAutoCommand();
} 

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::TestExit() {}