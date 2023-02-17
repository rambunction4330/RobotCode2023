#include "rmb/drive/DifferentialOdometry.h"
#include "ntcore_cpp.h"
#include "wpi/timestamp.h"

#include <iostream>

#include <units/time.h>

#include <networktables/DoubleArrayTopic.h>

#include <frc/kinematics/DifferentialDriveKinematics.h>

namespace rmb {

DifferentialOdometry::DifferentialOdometry(
  std::shared_ptr<LinearEncoder> left, std::shared_ptr<LinearEncoder> right,
  frc::DifferentialDriveKinematics& kinematics, 
  std::shared_ptr<frc::Gyro> gyroscope, std::string visionTable, 
  const frc::Pose2d& initalPose) 
  : leftEncoder(left), rightEncoder(right), gyro(gyroscope), poseEstimator(
      kinematics, gyro->GetRotation2d(), leftEncoder->getPosition(), 
      rightEncoder->getPosition(), initalPose, {0.02, 0.02, 0.01}, 
      {1.0, 1.0, 0.5}
    ) {

  // Get vision table. 
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable(visionTable);

  // Get positon and standard deviation topics.
  poseSubscriber = table->GetDoubleArrayTopic("pose").Subscribe({});
  stdDevSubscriber = table->GetDoubleArrayTopic("stdDev").Subscribe({});

  // Create positon callback
  poseListener = inst.AddListener(poseSubscriber, nt::EventFlags::kValueAll, 
    [this] (const nt::Event& event) {
      // Get timestamped data.
      nt::TimestampedDoubleArray rawData = poseSubscriber.GetAtomic();

      // Check data format.
      if (rawData.value.size() != 3) { return; }

      // Extract positon and time.
      frc::Pose2d pose = {units::meter_t(rawData.value[0]), 
                          units::meter_t(rawData.value[1]), 
                          units::radian_t(rawData.value[2])};

      units::second_t time = units::microsecond_t(rawData.time);

      // Lock mutex for thread saftey and add vision.
      std::lock_guard<std::mutex> lock(visionThreadMutex);
      poseEstimator.AddVisionMeasurement(pose, time);
    }
  );

  // Create standard deviation callback
  stdDevListener = inst.AddListener(stdDevSubscriber, nt::EventFlags::kValueAll, 
    [this] (const nt::Event& event) {
      // Get data from table.
      std::vector<double> rawData = stdDevSubscriber.Get();

      // Check data format
      if (rawData.size() != 3) { return; }

      // Lock mutex for thread saftey and add vision.
      std::lock_guard<std::mutex> lock(visionThreadMutex);
      poseEstimator.SetVisionMeasurementStdDevs({rawData[0], 
                                                 rawData[1], 
                                                 rawData[2]});
    }
  );
}

DifferentialOdometry::DifferentialOdometry(
  std::shared_ptr<LinearEncoder> left, std::shared_ptr<LinearEncoder> right,
  frc::DifferentialDriveKinematics& kinematics, 
  std::shared_ptr<frc::Gyro> gyroscope, const frc::Pose2d& initalPose) 
  : leftEncoder(left), rightEncoder(right), gyro(gyroscope), poseEstimator(
      kinematics, gyro->GetRotation2d(), leftEncoder->getPosition(), 
      rightEncoder->getPosition(), initalPose
    ) {}

DifferentialOdometry::~DifferentialOdometry() {
  // Remove listeners.
  nt::RemoveListener(poseListener);
  nt::RemoveListener(stdDevListener);
}

frc::Pose2d DifferentialOdometry::getPose() const {
  // Lock thread for saftey and get positon.
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.GetEstimatedPosition();
}


frc::Pose2d DifferentialOdometry::updatePose() {
  // Lock thread for saftey and updte position
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.Update(gyro->GetRotation2d(), 
                              leftEncoder->getPosition(), 
                              rightEncoder->getPosition());
}


void DifferentialOdometry::resetPose(const frc::Pose2d& pose) {
  // Lock thread for saftey and reset position
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.ResetPosition(gyro->GetRotation2d(), 
                              leftEncoder->getPosition(), 
                              rightEncoder->getPosition(),
                              pose);
}
} // namespace rmb