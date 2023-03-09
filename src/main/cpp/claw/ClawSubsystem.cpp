#include "claw/ClawSubsystem.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/InstantCommand.h>
#include <iostream> 

ClawSubsystem::ClawSubsystem() {
  clawMotor.ConfigFactoryDefault();
  clawMotor.ConfigStatorCurrentLimit({true, 60, 0, 0});
}

void ClawSubsystem::Periodic() {
  if (clawClosed) { 
    clawMotor.Set(currentAssistance);
  } else {
    clawMotor.Set(-0.8);
  }
}

void ClawSubsystem::setClawClosed(bool setClosed, double assistance) {
  clawClosed = setClosed;
  currentAssistance = assistance;
}

bool ClawSubsystem::getClawClosed() const {
  return clawClosed;
}

frc2::CommandPtr ClawSubsystem::getClosedCommand(bool setClosed, double assistance) {
  return frc2::InstantCommand([this, setClosed, assistance]() { 
           setClawClosed(setClosed, assistance); 
          }, {this}).ToPtr();
}
