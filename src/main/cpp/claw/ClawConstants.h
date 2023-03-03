#pragma once

#include <units/base.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/length.h>

namespace ClawSubsystemConstants {

units::meters_per_second_squared_t closeAssistRampMinAcceleration = 5.0_mps_sq;
units::meters_per_second_squared_t closeAssistRampMaxAcceleration = 10.0_mps_sq;

double closeBoostWithoutRamp = 0.0;
double closeBoostMax = 0.3;

}
