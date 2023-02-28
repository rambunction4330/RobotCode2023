// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/commands/JTurnCommand.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Transform2d.h"
#include "pathplanner/lib/PathPlannerTrajectory.h"
#include "units/velocity.h"

#include <pathplanner/lib/PathPlanner.h>

JTurnCommand::JTurnCommand(DriveSubsystem& driveSubsystem, 
                           pathplanner::RamseteAutoBuilder& autoBuilder) :
                        driveSubsystem(driveSubsystem), autoBuilder(autoBuilder) {
  AddRequirements(&driveSubsystem);
}

void JTurnCommand::Initialize() {
  // Check direction of robot movment
  bool reversed = driveSubsystem.getChassisSpeeds().vx < 0.0_mps;

  // Get speed of robot regardless of direction
  units::meters_per_second_t robotSpeed = units::math::abs(driveSubsystem.getChassisSpeeds().vx);

  // Find position of robot, putting the back in front if in reverse
  frc::Pose2d initalPose = driveSubsystem.getPose() + frc::Transform2d({0.0_m, 0.0_m}, reversed ? 180_deg : 0.0_deg);

  // Transforms from intial robot position between path points
  frc::Pose2d fwdStartPose = initalPose + frc::Transform2d({0.0_m, 0.0_m}, 0.0_deg);
  frc::Pose2d fwdEndPose = initalPose + frc::Transform2d({1.0_m, 1.0_m}, 90_deg);

  frc::Pose2d revStartPose = initalPose + frc::Transform2d({1.0_m, 1.0_m}, -90_deg);
  frc::Pose2d revEndPose = initalPose + frc::Transform2d({2.0_m, 0.0_m}, 0.0_deg);

  // Generate path rom points
  std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup {
    pathplanner::PathPlanner::generatePath(
      pathplanner::PathConstraints(3.75_mps, 2.0_mps_sq), reversed,
      std::vector<pathplanner::PathPoint> {
        pathplanner::PathPoint(fwdStartPose.Translation(), fwdStartPose.Rotation(), robotSpeed),
        pathplanner::PathPoint(fwdEndPose.Translation(), fwdEndPose.Rotation()),
      }
    ), pathplanner::PathPlanner::generatePath(
      pathplanner::PathConstraints(3.75_mps, 2.0_mps_sq), !reversed,
      std::vector<pathplanner::PathPoint> {
        pathplanner::PathPoint(revStartPose.Translation(), revStartPose.Rotation()),
        pathplanner::PathPoint(revEndPose.Translation(), revEndPose.Rotation(), -robotSpeed)
      }
    )
  };

  // Reassign path following command and initialize
  trajectoryCommand = autoBuilder.followPathGroup(trajectoryGroup).Unwrap();
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

