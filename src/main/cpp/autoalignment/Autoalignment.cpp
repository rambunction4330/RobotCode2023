#include "Autoalignment.h"

#include "drivetrain/DriveConstants.h"
#include "drivetrain/DriveSubsystem.h"
#include "frc/DriverStation.h"
#include <iostream>

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Transform2d.h"
#include "frc2/command/CommandPtr.h"

#include "pathplanner/lib/PathPlanner.h"
#include "pathplanner/lib/PathPlannerTrajectory.h"
#include "pathplanner/lib/PathPoint.h"
#include "pathplanner/lib/auto/BaseAutoBuilder.h"
#include "pathplanner/lib/auto/RamseteAutoBuilder.h"

const units::meter_t blueAlliancePastChargingStationThresholdX = 5.25_m;
const units::meter_t blueAllianceBeforeChargingStationThresholdX = 2.38_m;
const units::meter_t blueAlliancePastChargingStationThresholdYPositive = 4.54_m;
const units::meter_t blueAlliancePastChargingStationThresholdYNegative = 0.98_m;

const units::meter_t redAlliancePastChargingStationThresholdX = 11.34_m;
const units::meter_t redAllianceBeforeChargingStationThresholdX = 14.23_m;
const units::meter_t redAlliancePastChargingStationThresholdYPositive = 4.54_m;
const units::meter_t redAlliancePastChargingStationThresholdYNegative = 0.98_m;

const static frc::Pose2d pose8Negative =
    frc::Pose2d(frc::Translation2d(1.92_m, 0.45_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose8Center =
    frc::Pose2d(frc::Translation2d(1.92_m, 1.06_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose8Positive =
    frc::Pose2d(frc::Translation2d(1.92_m, 1.63_m), frc::Rotation2d(0.0_rad));

const static frc::Pose2d pose7Negative =
    frc::Pose2d(frc::Translation2d(1.92_m, 2.21_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose7Center =
    frc::Pose2d(frc::Translation2d(1.92_m, 2.75_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose7Positive =
    frc::Pose2d(frc::Translation2d(1.92_m, 3.31_m), frc::Rotation2d(0.0_rad));

const static frc::Pose2d pose6Negative =
    frc::Pose2d(frc::Translation2d(1.92_m, 3.86_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose6Center =
    frc::Pose2d(frc::Translation2d(1.92_m, 4.41_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose6Positive =
    frc::Pose2d(frc::Translation2d(1.92_m, 5.04_m), frc::Rotation2d(0.0_rad));

const static frc::Pose2d pose3Negative =
    frc::Pose2d(frc::Translation2d(14.61_m, 0.41_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose3Center =
    frc::Pose2d(frc::Translation2d(14.61_m, 1.06_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose3Positive =
    frc::Pose2d(frc::Translation2d(14.61_m, 1.63_m), frc::Rotation2d(0.0_rad));

const static frc::Pose2d pose2Negative =
    frc::Pose2d(frc::Translation2d(14.61_m, 2.21_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose2Center =
    frc::Pose2d(frc::Translation2d(14.61_m, 2.75_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose2Positive =
    frc::Pose2d(frc::Translation2d(14.61_m, 3.31_m), frc::Rotation2d(0.0_rad));

const static frc::Pose2d pose1Negative =
    frc::Pose2d(frc::Translation2d(14.61_m, 3.85_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose1Center =
    frc::Pose2d(frc::Translation2d(14.61_m, 4.41_m), frc::Rotation2d(0.0_rad));
const static frc::Pose2d pose1Positive =
    frc::Pose2d(frc::Translation2d(14.61_m, 5.04_m), frc::Rotation2d(0.0_rad));

constexpr const frc::Pose2d &
matchFieldLocationToPose(autoalignment::FieldLocation location) {
  switch (location) {
  case autoalignment::FieldLocationTag8Negative:
    return pose8Negative;
  case autoalignment::FieldLocationTag8Center:
    return pose8Center;
  case autoalignment::FieldLocationTag8Positive:
    return pose8Positive;
  case autoalignment::FieldLocationTag7Negative:
    return pose7Negative;
  case autoalignment::FieldLocationTag7Center:
    return pose7Center;
  case autoalignment::FieldLocationTag7Positive:
    return pose7Positive;
  case autoalignment::FieldLocationTag6Negative:
    return pose6Negative;
  case autoalignment::FieldLocationTag6Center:
    return pose6Center;
  case autoalignment::FieldLocationTag6Positive:
    return pose6Positive;
  case autoalignment::FieldLocationTag3Negative:
    return pose3Negative;
  case autoalignment::FieldLocationTag3Center:
    return pose3Center;
  case autoalignment::FieldLocationTag3Positive:
    return pose3Positive;
  case autoalignment::FieldLocationTag2Negative:
    return pose2Negative;
  case autoalignment::FieldLocationTag2Center:
    return pose2Center;
  case autoalignment::FieldLocationTag2Positive:
    return pose2Positive;
  case autoalignment::FieldLocationTag1Negative:
    return pose1Negative;
  case autoalignment::FieldLocationTag1Center:
    return pose1Center;
  case autoalignment::FieldLocationTag1Positive:
    return pose1Positive;
  default:
    return pose8Negative;
  }
}

frc2::CommandPtr
autoalignment::createAutoalignmentCommand(FieldLocation location,
                                          DriveSubsystem &driveSubsystem) {
  auto targetPose = matchFieldLocationToPose(location);
  auto team = frc::DriverStation::GetAlliance();
  std::vector<pathplanner::PathPoint> pathPoints;
  frc::Pose2d currentPose = driveSubsystem.getPose();

  if (team == frc::DriverStation::Alliance::kBlue) {
    auto currentPose = driveSubsystem.getPose();
    const units::meter_t chargingStationHorizontalBisectionLineY =
        (blueAlliancePastChargingStationThresholdYPositive +
         blueAlliancePastChargingStationThresholdYNegative) /
        2.0;

    const units::meter_t targetSideOfChargingStationYPos =
        currentPose.Y() > chargingStationHorizontalBisectionLineY
            ? blueAlliancePastChargingStationThresholdYPositive
            : blueAlliancePastChargingStationThresholdYNegative;

    if (currentPose.X() > blueAlliancePastChargingStationThresholdX) {
      const units::meter_t targetXPos =
          blueAlliancePastChargingStationThresholdX;

      pathPoints.push_back(pathplanner::PathPoint(
          frc::Translation2d(targetXPos, targetSideOfChargingStationYPos),
          frc::Rotation2d(0.0_deg)));
    }

    if (currentPose.X() > blueAllianceBeforeChargingStationThresholdX) {
      const units::meter_t targetXPos =
          blueAllianceBeforeChargingStationThresholdX;
      pathPoints.push_back(pathplanner::PathPoint(
          frc::Translation2d(targetXPos, targetSideOfChargingStationYPos),
          frc::Rotation2d(0.0_deg)));
    }

    pathPoints.push_back(pathplanner::PathPoint(
        frc::Translation2d(targetPose.X(), targetPose.Y()),
        frc::Rotation2d(targetPose.Rotation())));
  } else if (team == frc::DriverStation::Alliance::kRed) {
    auto currentPose = driveSubsystem.getPose();
    const units::meter_t chargingStationHorizontalBisectionLineY =
        (redAlliancePastChargingStationThresholdYPositive +
         redAlliancePastChargingStationThresholdYNegative) /
        2.0;

    const units::meter_t targetSideOfChargingStationYPos =
        currentPose.Y() > chargingStationHorizontalBisectionLineY
            ? redAlliancePastChargingStationThresholdYPositive
            : redAlliancePastChargingStationThresholdYNegative;

    if (currentPose.X() < redAlliancePastChargingStationThresholdX) {
      const units::meter_t targetXPos =
          redAlliancePastChargingStationThresholdX;

      pathPoints.push_back(pathplanner::PathPoint(
          frc::Translation2d(targetXPos, targetSideOfChargingStationYPos),
          frc::Rotation2d(180.0_deg)));
    }

    if (currentPose.X() < redAllianceBeforeChargingStationThresholdX) {
      const units::meter_t targetXPos =
          redAllianceBeforeChargingStationThresholdX;
      pathPoints.push_back(pathplanner::PathPoint(
          frc::Translation2d(targetXPos, targetSideOfChargingStationYPos),
          frc::Rotation2d(180.0_deg)));
    }

    pathPoints.push_back(pathplanner::PathPoint(
        frc::Translation2d(targetPose.X(), targetPose.Y()),
        frc::Rotation2d(targetPose.Rotation())));
  } else {
    std::cerr << "invalid alliance!" << std::endl;
  }

  pathplanner::PathPlannerTrajectory trajectory =
      pathplanner::PathPlanner::generatePath(
          pathplanner::PathConstraints(4_mps, 3_mps_sq), pathPoints);

  pathplanner::RamseteAutoBuilder builder(
      [&]() { return driveSubsystem.getPose(); },
      [&](frc::Pose2d initPose) { driveSubsystem.resetOdometry(initPose); },
      DriveConstants::ramseteController, DriveConstants::kinematics,
      [&](units::meters_per_second_t left, units::meters_per_second_t right) {
        driveSubsystem.driveWheelSpeeds(left, right);
      },
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>>(),
      {&driveSubsystem}, true);

  return builder.fullAuto(trajectory);
}
