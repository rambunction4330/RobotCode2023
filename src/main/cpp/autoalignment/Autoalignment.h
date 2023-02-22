#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Transform2d.h"
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>

#include <units/angle.h>
#include <units/base.h>

namespace autoalignment {
// locations
const frc::Pose2d pose8Negative =
    frc::Pose2d(frc::Translation2d(1.92_m, 0.45_m), frc::Rotation2d(0.0_rad));
const frc::Pose2d pose8Center =
    frc::Pose2d(frc::Translation2d(1.92_m, 1.06_m), frc::Rotation2d(0.0_rad));
const frc::Pose2d pose8Positive =
    frc::Pose2d(frc::Translation2d(1.92_m, 1.63_m), frc::Rotation2d(0.0_rad));

const frc::Pose2d pose7Negative =
    frc::Pose2d(frc::Translation2d(1.92_m, 2.21_m), frc::Rotation2d(0.0_rad));
const frc::Pose2d pose7Center =
    frc::Pose2d(frc::Translation2d(1.92_m, 2.75_m), frc::Rotation2d(0.0_rad));
const frc::Pose2d pose7Positive =
    frc::Pose2d(frc::Translation2d(1.92_m, 3.31_m), frc::Rotation2d(0.0_rad));

const frc::Pose2d pose6Negative =
    frc::Pose2d(frc::Translation2d(1.92_m, 3.86_m), frc::Rotation2d(0.0_rad));
const frc::Pose2d pose6Center =
    frc::Pose2d(frc::Translation2d(1.92_m, 4.41_m), frc::Rotation2d(0.0_rad));
const frc::Pose2d pose6Positive =
    frc::Pose2d(frc::Translation2d(1.92_m, 5.04_m), frc::Rotation2d(0.0_rad));

pathplanner::FollowPathWithEvents
createAutoalignmentCommand(frc::Pose2d targetPose);

} // namespace autoalignment
