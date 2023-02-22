#include "claw/ClawSubsystem.h"
#include "claw/ClawConstants.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/InstantCommand.h>
#include <iostream> 

ClawSubsystem::ClawSubsystem() {
  clawMotor->zeroPosition(0.0_deg);
  clawMotor->setPosition(ClawConstants::range.minPosition);
}

void ClawSubsystem::Periodic() {
  // std::cout << units::angle::to_string(units::degree_t(clawMotor->getPosition())) << std::endl;
}

void ClawSubsystem::setClawClosed(bool setClosed) {
  clawClosed = setClosed;
  clawMotor->setPosition(clawClosed ? 
                         ClawConstants::range.maxPosition : 
                         ClawConstants::range.minPosition);
}

bool ClawSubsystem::getClawClosed() const {
  return clawClosed;
}

void ClawSubsystem::toggleClaw() {
  setClawClosed(!clawClosed);
}

frc2::CommandPtr ClawSubsystem::getClosedCommand(bool setClosed) {
  return frc2::InstantCommand([this, setClosed]() { 
           setClawClosed(setClosed); 
          }, {this}).ToPtr();
}