#pragma once

#include "drivetrain/DriveSubsystem.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Transform2d.h"
#include "frc2/command/CommandPtr.h"
#include "pathplanner/lib/PathConstraints.h"
#include "pathplanner/lib/auto/RamseteAutoBuilder.h"
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>

#include <units/angle.h>
#include <units/base.h>

namespace autoalignment {
// locations

enum FieldLocation {
  FieldLocationTag8Negative,
  FieldLocationTag8Center,
  FieldLocationTag8Positive,

  FieldLocationTag7Negative,
  FieldLocationTag7Center,
  FieldLocationTag7Positive,

  FieldLocationTag6Negative,
  FieldLocationTag6Center,
  FieldLocationTag6Positive,

  FieldLocationTag3Negative,
  FieldLocationTag3Center,
  FieldLocationTag3Positive,

  FieldLocationTag2Negative,
  FieldLocationTag2Center,
  FieldLocationTag2Positive,

  FieldLocationTag1Negative,
  FieldLocationTag1Center,
  FieldLocationTag1Positive
};

frc2::CommandPtr
createAutoalignmentCommand(pathplanner::RamseteAutoBuilder& autoBuilder, pathplanner::PathConstraints constraints, FieldLocation location, DriveSubsystem& driveSubsystem);

class AutoAlignmentCommand : 
public frc2::CommandHelper<frc2::CommandBase, AutoAlignmentCommand> {
 public:
  AutoAlignmentCommand(DriveSubsystem& driveSubsystem, pathplanner::RamseteAutoBuilder& autoBuilder) :
      driveSubsystem(driveSubsystem), autoBuilder(autoBuilder) {}

  void Initialize() override {
    trajectoryCommand = createAutoalignmentCommand(autoBuilder, {1.0_mps, 1.5_mps_sq}, FieldLocationTag8Positive, driveSubsystem).Unwrap();
    trajectoryCommand->Initialize();
  }

  void Execute() override {
    trajectoryCommand->Execute();
  }

  void End(bool interrupted) override {
    trajectoryCommand->End(interrupted);
  }

  bool IsFinished() override {
    return trajectoryCommand->IsFinished();
  }

 private:
  DriveSubsystem& driveSubsystem;
  pathplanner::RamseteAutoBuilder& autoBuilder;
  std::unique_ptr<frc2::Command> trajectoryCommand = frc2::PrintCommand("ERROR\n").ToPtr().Unwrap();
};
} // namespace autoalignment
