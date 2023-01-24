#pragma once

#include <frc/geometry/Pose2d.h>

namespace rmb {
  
/**
 * Common interface to robot odometry (where the robot is on the field).
 * @see DifferentialOdometry
 */
class DriveOdometry {
public:
  /**
   * Common Interface for getting the current estimated robot position.
   * 
   * @return The current position of the robot
   */
  virtual frc::Pose2d getPose() const = 0;

  /**
   * Common Interface for updating the estimated robot position.
   * 
   * @return The updated position
   */
  virtual frc::Pose2d updatePose() = 0;

  /**
   * Common Interface for re-setting the estimated position.
   * 
   * @return The pose as a Pose2d
   */
  virtual void resetPose(const frc::Pose2d &pose = frc::Pose2d()) = 0;
};
} // namespace rmb