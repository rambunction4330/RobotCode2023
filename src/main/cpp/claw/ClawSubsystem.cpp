#include "claw/ClawSubsystem.h"

#include <frc/trajectory/TrapezoidProfile.h>

#include <iostream>

ClawSubsystem::ClawSubsystem() {
    clawMotor->zeroPosition(0.0_deg);
}

void ClawSubsystem::Periodic() {}

void ClawSubsystem::setClawClosed(bool isClosed) {
  clawClosed = isClosed;
  clawMotor->setPosition(clawClosed ? ClawConstants::range.maxPosition : ClawConstants::range.minPosition);
}

bool ClawSubsystem::getClawClosed() const {
  return clawClosed;
}

void ClawSubsystem::toggleClaw() {
  setClawClosed(!clawClosed);
}