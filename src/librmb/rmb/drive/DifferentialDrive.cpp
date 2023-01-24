#include "rmb/drive/DifferentialDrive.h"

#include <frc/drive/DifferentialDrive.h>

namespace rmb {

DifferentialDrive::DifferentialDrive(std::shared_ptr<LinearVelocityController> l, 
                                     std::shared_ptr<LinearVelocityController> r,
                                     frc::DifferentialDriveKinematics k) : 
                                     left(l), right(r), kinematics(k) {}

void DifferentialDrive::arcadeDrive(double xSpeed, double zRotation) {
  auto wheelSpeeds = frc::DifferentialDrive::ArcadeDriveIK(xSpeed, zRotation, false);
  left->setPower(wheelSpeeds.left);
  right->setPower(wheelSpeeds.right);
}

void DifferentialDrive::curvatureDrive(double xSpeed, double zRotation, bool turnInPlace) {
  auto wheelSpeeds = frc::DifferentialDrive::CurvatureDriveIK(xSpeed, zRotation, turnInPlace);
  left->setPower(wheelSpeeds.left);
  right->setPower(wheelSpeeds.right);
}

void DifferentialDrive::tankDrive(double leftSpeed, double rightSpeed) {
  auto wheelSpeeds = frc::DifferentialDrive::TankDriveIK(leftSpeed, rightSpeed);
  left->setPower(wheelSpeeds.left);
  right->setPower(wheelSpeeds.right);
}

 void DifferentialDrive::driveWheelSpeeds(units::meters_per_second_t leftVelocity, units::meters_per_second_t rightVelocity) {
  left->setVelocity(leftVelocity);
  right->setVelocity(rightVelocity);
 }

void DifferentialDrive::driveWheelSpeeds(frc::DifferentialDriveWheelSpeeds wheelSpeeds) {
  left->setVelocity(wheelSpeeds.left);
  right->setVelocity(wheelSpeeds.right);
}

void DifferentialDrive::driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds) {
  driveWheelSpeeds(kinematics.ToWheelSpeeds(chassisSpeeds));
}

} // namespace rmb