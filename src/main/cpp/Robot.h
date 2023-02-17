#pragma once

#include <frc/TimedRobot.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
public:
void RobotInit() override;
void RobotPeriodic() override;

void TeleopInit() override;
void TeleopPeriodic() override;
void TeleopExit() override;

void AutonomousInit() override;
void AutonomousPeriodic() override;
void AutonomousExit() override;

void TestInit() override;
void TestPeriodic() override;
void TestExit() override;

private:
  RobotContainer container;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
