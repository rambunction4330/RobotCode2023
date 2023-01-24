#include "rmb/drive/DifferentialOdometry.h"

#include <units/time.h>

#include <networktables/DoubleArrayTopic.h>

#include <frc/kinematics/DifferentialDriveKinematics.h>

namespace rmb {

DifferentialOdometry::DifferentialOdometry(
  std::shared_ptr<LinearEncoder> left, std::shared_ptr<LinearEncoder> right,
  frc::DifferentialDriveKinematics& kinematics, std::shared_ptr<frc::Gyro> gyroscope, 
  std::shared_ptr<nt::NetworkTable> visionTable, const frc::Pose2d& initalPose
) : leftEncoder(left), rightEncoder(right), gyro(gyroscope), 
    poseEstimator(kinematics, gyro->GetRotation2d(), leftEncoder->getPosition(), 
                  rightEncoder->getPosition(), initalPose) {

  visionListener = visionTable->AddListener(
    nt::EventFlags::kPublish, 
    [this](nt::NetworkTable* table, std::string_view key, const nt::Event& event){

    if (key == "pose") {
      nt::TimestampedDoubleArray rawData = table->GetDoubleArrayTopic(key).Subscribe({}).GetAtomic();

      if (rawData.value.size() != 3) { return; }

      frc::Pose2d pose = {units::meter_t(rawData.value[0]), units::meter_t(rawData.value[1]), units::radian_t(rawData.value[2])};
      units::second_t time = units::microsecond_t(rawData.time);

      std::lock_guard<std::mutex> lock(visionThreadMutex);
      poseEstimator.AddVisionMeasurement(pose, time);
    }

    if (key == "stdDev") {
      std::vector<double> rawData = table->GetDoubleArrayTopic(key).Subscribe({}).Get();

      if (rawData.size() != 3) { return; }

      std::lock_guard<std::mutex> lock(visionThreadMutex);
      poseEstimator.SetVisionMeasurementStdDevs({rawData[0], rawData[1], rawData[2]});
    }
  });
}

DifferentialOdometry::~DifferentialOdometry() {
  nt::RemoveListener(visionListener);
}

DifferentialOdometry::DifferentialOdometry(
  std::shared_ptr<LinearEncoder> left, std::shared_ptr<LinearEncoder> right,
  frc::DifferentialDriveKinematics& kinematics, std::shared_ptr<frc::Gyro> gyroscope, 
  const frc::Pose2d& initalPose
) : leftEncoder(left), rightEncoder(right), gyro(gyroscope), 
    poseEstimator(kinematics, gyro->GetRotation2d(), leftEncoder->getPosition(), 
                 rightEncoder->getPosition(), initalPose) {}

frc::Pose2d DifferentialOdometry::getPose() const {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.GetEstimatedPosition();
}


frc::Pose2d DifferentialOdometry::updatePose() {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.Update(gyro->GetRotation2d(), 
                              leftEncoder->getPosition(), 
                              rightEncoder->getPosition());
}


void DifferentialOdometry::resetPose(const frc::Pose2d& pose) {
  // leftEncoder->zeroPosition();
  // rightEncoder->zeroPosition();
  gyro->Reset();
  

  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.ResetPosition(gyro->GetRotation2d(), 
                              leftEncoder->getPosition(), 
                              rightEncoder->getPosition(),
                              pose);
}
} // namespace rmb