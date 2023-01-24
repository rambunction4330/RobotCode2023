
#pragma once

#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/button/Trigger.h>

namespace rmb {

class LogitechGamepad : public frc2::CommandGenericHID {
public:

  struct Axes {
    constexpr static int leftX = 1;
    constexpr static int leftY = 0;
    constexpr static int leftTrigger = 2;

    constexpr static int rightX = 5;
    constexpr static int rightY = 4;
    constexpr static int rightTrigger = 3;
  };

  struct Buttons {
    constexpr static int leftStick = 9;
    constexpr static int rightStick = 10;

    constexpr static int rightBumper = 6;
    constexpr static int leftBumper = 5;

    constexpr static int X = 3;
    constexpr static int Y = 4;
    constexpr static int A = 1;
    constexpr static int B = 2;

    constexpr static int backButton = 7;
    constexpr static int startButton = 8;
  };

  LogitechGamepad(int channel, double deadZone = 0.0, bool squareOutputs = false) : 
    frc2::CommandGenericHID(channel), deadZone(deadZone), squareOutputs(squareOutputs) {}

  double GetLeftX() const {
  double raw = -GetRawAxis(Axes::leftX);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}
frc2::Trigger LeftXLessThan(double threshold) const { return AxisLessThan(Axes::leftX, threshold); }
frc2::Trigger LeftXGreaterThan(double threshold) const { return AxisGreaterThan(Axes::leftX, threshold); }

double GetLeftY() const { 
  double raw = GetRawAxis(Axes::leftY);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}

frc2::Trigger LeftYLessThan(double threshold) const { return AxisLessThan(Axes::leftY, threshold); }
frc2::Trigger LeftYgreaterThan(double threshold) const { return AxisGreaterThan(Axes::leftY, threshold); }

bool GetLeftStickButton() const { return GetRawButton(Buttons::leftStick); }
bool GetLeftStickButtonPressed() { return GetRawButtonPressed(Buttons::leftStick); }
bool GetLeftStickButtonReleased() { return GetRawButtonPressed(Buttons::leftStick); }
frc2::Trigger LeftStickButton() const { return Button(Buttons::leftStick); }

double GetRightX() const {
  double raw = -GetRawAxis(Axes::rightX);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}

frc2::Trigger RightXLessThan(double threshold) const { return AxisLessThan(Axes::rightX, threshold); }
frc2::Trigger RightXgreaterThan(double threshold) const { return AxisGreaterThan(Axes::rightX, threshold); }

double GetRightY() const {
  double raw = GetRawAxis(Axes::rightY);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}

frc2::Trigger RightYLessThan(double threshold) const { return AxisLessThan(Axes::rightY, threshold); }
frc2::Trigger RightYGearterThan(double threshold) const { return AxisGreaterThan(Axes::rightY, threshold); }

bool GetRightStickButton() const { return GetRawButton(Buttons::rightStick); }
bool GetRightStickButtonPressed() { return GetRawButtonPressed(Buttons::rightStick); }
bool GetRightStickButtonReleased() { return GetRawButtonPressed(Buttons::rightStick); }
frc2::Trigger RightStickButton() const { return Button(Buttons::rightStick); }

double GetLeftTrigger() const {
double raw = GetRawAxis(Axes::leftTrigger);
if (abs(raw) < deadZone) { return 0.0; }
return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}
frc2::Trigger LeftTriggerLessThan(double threshold) const { return AxisLessThan(Axes::leftTrigger, threshold); }
frc2::Trigger LeftTriggergreaterThan(double threshold) const { return AxisGreaterThan(Axes::leftTrigger, threshold); }

double GetRightTrigger() const {
  double raw = GetRawAxis(Axes::rightTrigger);
  if (abs(raw) < deadZone) { return 0.0; }
  return squareOutputs ? std::copysign(raw * raw, raw) : raw;
}
frc2::Trigger RightTriggerLessThan(double threshold) const { return AxisLessThan(Axes::rightTrigger, threshold); }
frc2::Trigger RightTriggerGreaterThan(double threshold) const { return AxisGreaterThan(Axes::rightTrigger, threshold); }

bool GetLeftBumper() const { return GetRawButton(Buttons::leftBumper); }
bool GetLeftBumperPressed() { return GetRawButtonPressed(Buttons::leftBumper); }
bool GetLeftBumperReleased() { return GetRawButtonPressed(Buttons::leftBumper); }
frc2::Trigger LeftBumper() const { return Button(Buttons::leftBumper); }

bool GetRightBumper() const { return GetRawButton(Buttons::rightBumper); }
bool GetRightBumperPressed() { return GetRawButtonPressed(Buttons::rightBumper); }
bool GetRightBumperReleased() { return GetRawButtonPressed(Buttons::rightBumper); }
frc2::Trigger RightBumper() const { return Button(Buttons::rightBumper); }

bool GetX() const { return GetRawButton(Buttons::X); }
bool GetXPressed() { return GetRawButtonPressed(Buttons::X); }
bool GetXReleased() { return GetRawButtonReleased(Buttons::X); }
frc2::Trigger X() const { return Button(Buttons::X); }

bool GetY() const { return GetRawButton(Buttons::Y); }
bool GetYPressed() { return GetRawButtonPressed(Buttons::Y); }
bool GetYReleased() { return GetRawButtonReleased(Buttons::Y); }
frc2::Trigger Y() const { return Button(Buttons::Y); }

bool GetA() const { return GetRawButton(Buttons::A); }
bool GetAPressed() { return GetRawButtonPressed(Buttons::A); }
bool GetAReleased() { return GetRawButtonReleased(Buttons::A); }
frc2::Trigger A() const { return Button(Buttons::A); }

bool GetB() const { return GetRawButton(Buttons::B); }
bool GetBPressed() { return GetRawButtonPressed(Buttons::B); }
bool GetBReleased() { return GetRawButtonReleased(Buttons::B); }
frc2::Trigger B() const { return Button(Buttons::B); }

bool GetBackButton() const { return GetRawButton(Buttons::backButton); }
bool GetBackButtonPressed() { return GetRawButtonPressed(Buttons::backButton); }
bool GetBackButtonReleased() { return GetRawButtonReleased(Buttons::backButton); }
frc2::Trigger BackButton() const { return Button(Buttons::backButton); }

bool GetStartButton() const { return GetRawButton(Buttons::startButton); }
bool GetStartButtonPressed() { return GetRawButtonPressed(Buttons::startButton); }
bool GetStartButtonReleased() { return GetRawButtonReleased(Buttons::startButton); }
frc2::Trigger StartButton() const { return Button(Buttons::startButton); }

private:
  double deadZone;
  bool squareOutputs;
};
} // namespace rmb