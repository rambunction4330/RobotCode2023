#pragma once

#include <units/base.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/length.h>

namespace ClawSubsystemConstants {

units::meters_per_second_squared_t closeAssistRampMinAcceleration = 0.5_mps_sq;
units::meters_per_second_squared_t closeAssistRampMaxAcceleration = 1.5_mps_sq;

double closeBoostWithoutRamp = 0.2;
double closeBoostMax = 0.8;

}
