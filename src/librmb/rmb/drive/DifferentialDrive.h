
#pragma once

#include <memory>

#include <units/velocity.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#include "rmb/motorcontrol/LinearVelocityController.h"

namespace rmb {
class DifferentialDrive {
public:

  DifferentialDrive(std::shared_ptr<LinearVelocityController> left, 
                    std::shared_ptr<LinearVelocityController> right,
                    frc::DifferentialDriveKinematics kinematics);

  void arcadeDrive(double xSpeed, double zRotation);
  void curvatureDrive(double xSpeed, double zRotation, bool turnInPlace);
  void tankDrive(double leftSpeed, double rightSpeed);

  void driveWheelSpeeds(units::meters_per_second_t leftVelocity, units::meters_per_second_t rightVelocity);
  void driveWheelSpeeds(frc::DifferentialDriveWheelSpeeds wheelSpeeds);
  void driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds);

private:
  std::shared_ptr<LinearVelocityController> left, right;
  frc::DifferentialDriveKinematics kinematics;
};
} // namespace rmb