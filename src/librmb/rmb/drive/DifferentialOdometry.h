
#pragma once

#include <memory>
#include <string>

#include <units/length.h>

#include <networktables/NetworkTableInstance.h>

#include <frc/geometry/Pose2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>

#include "rmb/drive/DriveOdometry.h"
#include "rmb/motorcontrol/feedback/LinearEncoder.h"

namespace rmb {

/**
 * Odometry for a differential drive based on encoder and gyroscope data. It 
 * can also optionaly incorperate vision based odometry over NetworkTables.
 */
class DifferentialOdometry : public DriveOdometry {
public:

  /**
   * Constructs a DiffernetialDriveOdometry object using encoder, gyro and 
   * vision data.
   * 
   * @param leftEncoder  A shared pointer to the left side encoder. If there 
   *                     are multiple, the back wheel is usually prefered.
   * 
   * @param rightEncoder A shared pointer to the right side encoder. If there 
   *                     are multiple, the back wheel is usually prefered.
   * 
   * @param trackWidth   The distance between the left and right side wheels
   *                     for kinematics.
   * 
   * @param gyro         A shared pointer to a gyroscopefor determining the 
   *                     robots heading.
   * 
   * @param visionTable  A shared pointer to the `NetworkTable` holding a vision 
   *                     based estimation of robot position. This table must 
   *                     include a `DoubleArrayTopic` named "pose". The array 
   *                     must have three elements ordered X, Y, Heading. X and 
   *                     Y must be in meters and heading must be in radians. 
   *                     The table may also include a `DoubleArrayTopic` named 
   *                     "stdDev" as how much this estimation can be trusted. 
   *                     The array is also ordered X, Y, Heading with X and Y 
   *                     in meters and Heading in radians. When using the 
   *                     "stdDev" topic, always update the standard deviations
   *                     befor the position.
   * 
   * @param initalPose   Inital position of the robot.
   */ 
  DifferentialOdometry(std::shared_ptr<LinearEncoder> leftEncoder, 
                       std::shared_ptr<LinearEncoder> rightEncoder,
                       frc::DifferentialDriveKinematics& kinematics,
                       std::shared_ptr<frc::Gyro> gyro,
                       std::shared_ptr<nt::NetworkTable> visionTable,
                       const frc::Pose2d& initalPose = frc::Pose2d());

  /**
   * Constructs a DiffernetialDriveOdometry object using only encoder and gyro 
   * data.
   * 
   * @param leftEncoder  A shared pointer to the left side encoder. If there 
   *                     are multiple, the back wheel is usually prefered.
   * 
   * @param rightEncoder A shared pointer to the right side encoder. If there 
   *                     are multiple, the back wheel is usually prefered.
   * 
   * @param trackWidth   The distance between the left and right side wheels
   *                     for kinematics.
   * 
   * @param gyro         A shared pointer to a gyroscopefor determining the 
   *                     robots heading.
   * 
   * @param initalPose   Inital position of the robot.
   */ 
  DifferentialOdometry(std::shared_ptr<LinearEncoder> leftEncoder, 
                       std::shared_ptr<LinearEncoder> rightEncoder,
                       frc::DifferentialDriveKinematics& kinematics,
                       std::shared_ptr<frc::Gyro> gyro,
                       const frc::Pose2d& initalPose = frc::Pose2d());

  ~DifferentialOdometry();

  /**
   * Returns the current poition without modifying it.
   * 
   * @return The current estimated position of the robot.
   */
  frc::Pose2d getPose() const override;

  /**
   * Updates the current position of the robot using encoder and gyroscope 
   * data. Vision estimations are updated on a separate thread generated at object construction
   * 
   * @return The updated position.
   */
  frc::Pose2d updatePose() override;

  /**
   * Resets the estimated robot poition.
   * 
   * @return The position to reset the robots estimated position to.
   */
  void resetPose(const frc::Pose2d& pose = frc::Pose2d()) override;

private:
  std::shared_ptr<LinearEncoder> leftEncoder, rightEncoder;
  std::shared_ptr<frc::Gyro> gyro;
  frc::DifferentialDrivePoseEstimator poseEstimator;

  // Vision Handleing
  NT_Listener visionListener;
  mutable std::mutex visionThreadMutex;
};
} // namespace rmb