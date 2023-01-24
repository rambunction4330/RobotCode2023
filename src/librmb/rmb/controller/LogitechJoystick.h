
#pragma once

#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/button/Trigger.h>

namespace rmb {

class LogitechJoystick : public frc2::CommandGenericHID {
public:

  struct Axes {
    constexpr static int X = 1;
    constexpr static int Y = 0;
    constexpr static int twist = 2;
    constexpr static int throttle = 3;
  };

  LogitechJoystick(int channel, double deadZone = 0.0, bool squareOutputs = false):
    frc2::CommandGenericHID(channel), deadZone(deadZone), squareOutputs(squareOutputs) {};

double GetX() const {
  double raw = -GetRawAxis(Axes::X);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}

frc2::Trigger XLessThan(double threshold) const { return AxisLessThan(Axes::X, threshold); }
frc2::Trigger XMoreThan(double threshold) const { return AxisGreaterThan(Axes::X, threshold); }

double GetY() const {
  double raw = GetRawAxis(Axes::Y);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}

frc2::Trigger YLessThan(double threshold) const { 
  return AxisLessThan(Axes::Y, threshold); 
}

frc2::Trigger YMoreThan(double threshold) const { return AxisGreaterThan(Axes::Y, threshold); }

double GetTwist() const {
  double raw = GetRawAxis(Axes::twist);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}

frc2::Trigger TwistLessThan(double threshold) const { return AxisLessThan(Axes::twist, threshold); }
frc2::Trigger TwistMoreThan(double threshold) const { return AxisGreaterThan(Axes::twist, threshold); }

double GetThrottle() const {
  double raw = -GetRawAxis(Axes::throttle);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}

frc2::Trigger ThrottleLessThan(double threshold) const { return AxisLessThan(Axes::throttle, threshold); }
frc2::Trigger ThrottleMoreThan(double threshold) const{ return AxisGreaterThan(Axes::throttle, threshold); }

bool GetTrigger() const { return GetRawButton(1); }
bool GetTriggerPressed() { return GetRawButtonPressed(1); }
bool GetTriggerReleased() { return GetRawButtonReleased(1); }
frc2::Trigger Trigger() const { return Button(1); }

private:
  double deadZone;
  bool squareOutputs;
};
} // namespace rmb