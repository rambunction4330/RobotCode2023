#include "claw/ClawSubsystem.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/InstantCommand.h>

ClawSubsystem::ClawSubsystem() {
    clawMotor->zeroPosition(35.0_deg);
    clawMotor->setPosition(45.0_deg);
}

void ClawSubsystem::Periodic() {}

void ClawSubsystem::setClawClosed(bool setClosed) {
  clawClosed = setClosed;
  clawMotor->setPosition(clawClosed ? ClawConstants::range.maxPosition : ClawConstants::range.minPosition);
}

bool ClawSubsystem::getClawClosed() const {
  return clawClosed;
}

void ClawSubsystem::toggleClaw() {
  setClawClosed(!clawClosed);
}

frc2::CommandPtr ClawSubsystem::getClosedCommand(bool setClosed) {
  return frc2::InstantCommand([this, setClosed]() { setClawClosed(setClosed); }, {this}).ToPtr();
}