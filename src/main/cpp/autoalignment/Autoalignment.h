#pragma once

#include "drivetrain/DriveSubsystem.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Transform2d.h"
#include "frc2/command/CommandPtr.h"
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
  FieldLocationTag6Positive
};

frc2::CommandPtr
createAutoalignmentCommand(FieldLocation location, DriveSubsystem& driveSubsystem);

} // namespace autoalignment
