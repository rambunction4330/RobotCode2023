#include "claw/ClawSubsystem.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/InstantCommand.h>
#include <iostream> 

ClawSubsystem::ClawSubsystem() {
  clawMotor.RestoreFactoryDefaults();
}

void ClawSubsystem::Periodic() {
  if (clawClosed) { 
    clawMotor.Set(currentAssistance); 
  } else {
    clawMotor.Set(-0.4);
  }
}

void ClawSubsystem::setClawClosed(bool setClosed, double assistance) {
  clawClosed = setClosed;
  currentAssistance = assistance;
}

bool ClawSubsystem::getClawClosed() const {
  return clawClosed;
}

frc2::CommandPtr ClawSubsystem::getClosedCommand(bool setClosed) {
  return frc2::InstantCommand([this, setClosed]() { 
           setClawClosed(setClosed); 
          }, {this}).ToPtr();
}
