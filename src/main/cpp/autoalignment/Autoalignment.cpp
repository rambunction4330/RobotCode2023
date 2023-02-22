#include "Autoalignment.h"

#include "drivetrain/DriveConstants.h"
#include "drivetrain/DriveSubsystem.h"
#include "frc/DriverStation.h"
#include <iostream>

#include "frc/geometry/Transform2d.h"
#include "frc2/command/CommandPtr.h"
#include "pathplanner/lib/PathPlanner.h"
#include "pathplanner/lib/PathPlannerTrajectory.h"
#include "pathplanner/lib/PathPoint.h"
#include "pathplanner/lib/auto/BaseAutoBuilder.h"
#include "pathplanner/lib/auto/RamseteAutoBuilder.h"

const double blueAlliancePastChargingStationThresholdX = 5.45;
const double blueAlliancePastChargingStationThresholdYPositive = 4.40;
const double blueAlliancePastChargingStationThresholdYNegative = 1.13;
const double blueAllianceBeforeChargingStationThresholdX = 1.45;

/**TODO: this*/
frc2::CommandPtr
autoalignment::createAutoalignmentCommand(frc::Pose2d targetPose,
                                          DriveSubsystem &driveSubsystem) {
  auto team = frc::DriverStation::GetAlliance();
  std::vector<pathplanner::PathPoint> pathPoints;


  if (team == frc::DriverStation::Alliance::kBlue) {
    auto currentPose = driveSubsystem.getPose();
    if (currentPose.X()() > blueAlliancePastChargingStationThresholdX) {
      const double yMidpoint =
          (blueAlliancePastChargingStationThresholdYNegative +
           blueAlliancePastChargingStationThresholdYPositive) /
          2.0;

      if (currentPose.Y()() > yMidpoint) {
        pathplanner::PathPoint pointA(
            frc::Translation2d(
                units::meter_t(blueAlliancePastChargingStationThresholdX),
                units::meter_t(
                    blueAlliancePastChargingStationThresholdYPositive)),
            frc::Rotation2d(0.0_deg));
        pathplanner::PathPoint pointB(
            frc::Translation2d(
                units::meter_t(blueAllianceBeforeChargingStationThresholdX),
                units::meter_t(
                    blueAlliancePastChargingStationThresholdYNegative)),
            frc::Rotation2d(0.0_deg));
        pathPoints.push_back(pointA);
        pathPoints.push_back(pointB);

      } else {
        pathplanner::PathPoint pointA(
            frc::Translation2d(
                units::meter_t(blueAlliancePastChargingStationThresholdX),
                units::meter_t(
                    blueAlliancePastChargingStationThresholdYPositive)),
            frc::Rotation2d(0.0_deg));
        pathplanner::PathPoint pointB(
            frc::Translation2d(
                units::meter_t(blueAllianceBeforeChargingStationThresholdX),
                units::meter_t(
                    blueAlliancePastChargingStationThresholdYPositive)),
            frc::Rotation2d(0.0_deg));
        pathPoints.push_back(pointA);
        pathPoints.push_back(pointB);
      }
    }
  } else if (team == frc::DriverStation::Alliance::kRed) {

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
