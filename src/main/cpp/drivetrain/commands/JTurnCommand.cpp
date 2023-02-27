// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/commands/JTurnCommand.h"

#include <pathplanner/lib/PathPlanner.h>

JTurnCommand::JTurnCommand(DriveSubsystem& driveSubsystem, 
                           pathplanner::RamseteAutoBuilder& autoBuilder) :
                        driveSubsystem(driveSubsystem), autoBuilder(autoBuilder) {}

void JTurnCommand::Initialize() {
  // Get data about the current stat eof the robot.
  frc::Pose2d robotPose = driveSubsystem.getPose();
  frc::ChassisSpeeds robotSpeeds = driveSubsystem.getChassisSpeeds();
  bool reverse = robotSpeeds.vx < 0.0_mps;

  // Transforms from intial robot position to the two path points.
  frc::Transform2d midTransform {{0.8_m, 0.8_m}, 90_deg};
  frc::Transform2d endTransform {{1.6_m, 0.0_m}, 180_deg};

  // Generate path
  pathplanner::PathPlannerTrajectory trajectory = pathplanner::PathPlanner::generatePath(
    pathplanner::PathConstraints(3_mps, 2_mps_sq), reverse,
    std::vector<pathplanner::PathPoint> {
      pathplanner::PathPoint(robotPose.Translation(), robotPose.Rotation(), robotSpeeds.vx),
      pathplanner::PathPoint((robotPose + midTransform).Translation(), (robotPose + midTransform).Rotation()),
      pathplanner::PathPoint((robotPose + endTransform).Translation(), (robotPose + endTransform).Rotation())
    }
  );

  trajectoryCommand = autoBuilder.followPath(trajectory).Unwrap();
  trajectoryCommand->Initialize();
}

void JTurnCommand::Execute() {
  trajectoryCommand->Execute();
}

void JTurnCommand::End(bool interrupted) {
  trajectoryCommand->End(interrupted);
}

bool JTurnCommand::IsFinished() {
  return trajectoryCommand->IsFinished();
}

