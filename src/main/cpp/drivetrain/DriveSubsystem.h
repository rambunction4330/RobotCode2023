// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <thread>

#include <units/acceleration.h>

#include <frc2/command/CommandPtr.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <AHRS.h>

#include <pathplanner/lib/PathPlannerTrajectory.h>

#include <rmb/motorcontrol/feedback/LinearVelocityFeedbackController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>
#include <rmb/drive/DifferentialDrive.h>
#include <rmb/drive/DifferentialOdometry.h>

#include <rmb/controller/LogitechGamepad.h>
#include <rmb/controller/LogitechJoystick.h>
#include <frc2/command/PrintCommand.h>

#include "DriveConstants.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  // Teleop Drive Fucntions
  void arcadeDrive(double xSpeed, double zRotation);
  void arcadeDrive(const rmb::LogitechJoystick& joystick);
  void arcadeDrive(const rmb::LogitechGamepad& gamepad);
  frc2::CommandPtr arcadeDriveCommand(const rmb::LogitechJoystick& joystick);
  frc2::CommandPtr arcadeDriveCommand(const rmb::LogitechGamepad& gamepad);

  void curvatureDrive(double xSpeed, double zRotation, bool turnInPlace);
  void curvatureDrive(const rmb::LogitechJoystick& stick);
  void curvatureDrive(const rmb::LogitechGamepad& gamepad);
  frc2::CommandPtr curvatureDriveCommand(const rmb::LogitechJoystick& stick);
  frc2::CommandPtr curvatureDriveCommand(const rmb::LogitechGamepad& gamepad);

  void tankDrive(double leftSpeed, double rightSpeed);
  void tankDrive(const rmb::LogitechJoystick& left, const rmb::LogitechJoystick& right);
  void tankDrive(const rmb::LogitechGamepad& gamepad);
  frc2::CommandPtr tankDirveCommand(const rmb::LogitechJoystick& left, const rmb::LogitechJoystick& right);
  frc2::CommandPtr tankDriveCommand(const rmb::LogitechGamepad& gamepad);

  // Odometry Fucntions
  void resetOdometry(frc::Pose2d pose = frc::Pose2d());
  frc::Pose2d getPose() const;
  frc::DifferentialDriveWheelSpeeds getWheelSpeeds() const;
  frc::ChassisSpeeds getChassisSpeeds() const;


  // Auto Driving
  void stop();
  void driveWheelSpeeds(units::meters_per_second_t left, units::meters_per_second_t right);
  void driveWheelSpeeds(frc::DifferentialDriveWheelSpeeds wheelSpeeds);
  void driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds);

  // Balancing
  units::radian_t getRobotPitch();
  frc2::CommandPtr getBalanceCommand();



  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  std::shared_ptr<rmb::LinearVelocityFeedbackController> left {
    rmb::asLinear(
      (std::shared_ptr<rmb::AngularVelocityFeedbackController>)
      std::make_shared<rmb::SparkMaxVelocityController>(
        DriveConstants::leftLeader, DriveConstants::pidConfig, 
        DriveConstants::profileConfig, DriveConstants::feedbackConfig, 
        std::initializer_list<const rmb::SparkMaxVelocityController::MotorConfig>{DriveConstants::leftFollower}
      ),
      DriveConstants::wheelDiameter / 2.0_rad
    )
  };

  std::shared_ptr<rmb::LinearVelocityFeedbackController> right {
    rmb::asLinear(
      (std::shared_ptr<rmb::AngularVelocityFeedbackController>)
      std::make_shared<rmb::SparkMaxVelocityController>(
        DriveConstants::rightLeader, DriveConstants::pidConfig, 
        DriveConstants::profileConfig, DriveConstants::feedbackConfig, 
        std::initializer_list<const rmb::SparkMaxVelocityController::MotorConfig>{DriveConstants::rightFollower}
      ), 
      DriveConstants::wheelDiameter / 2.0_rad
    )
  };

  rmb::DifferentialDrive drive { 
    left, right, DriveConstants::kinematics 
  };

  std::shared_ptr<AHRS> gyro = std::make_shared<AHRS>(DriveConstants::gyroPort); 

  rmb::DifferentialOdometry odometry { 
    left, right, 
    DriveConstants::kinematics, 
    gyro,
    // DriveConstants::visionTableString
  };

  frc::Field2d displayFeild; 
};
