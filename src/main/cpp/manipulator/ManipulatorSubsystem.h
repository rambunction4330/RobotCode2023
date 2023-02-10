
#pragma once

#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rmb/motorcontrol/feedback/AngularPositionFeedbackController.h>
#include <rmb/motorcontrol/feedback/LinearPositionFeedbackController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

#include "manipulator/ManipulatorConstants.h"

class ManipulatorSubsystem : public frc2::SubsystemBase {
 public:

  /*********************
   * Manipulator State *
   *********************/
  struct ManipulatorState {
    units::meter_t elevatorHeight;
    units::radian_t armAngle;
  };

  constexpr static ManipulatorSubsystem::ManipulatorState compactState {11_in, 92.5_deg};
  constexpr static ManipulatorSubsystem::ManipulatorState highState {38.0_in, 10.0_deg};
  constexpr static ManipulatorSubsystem::ManipulatorState midState {20_in, 60_deg};
  constexpr static ManipulatorSubsystem::ManipulatorState lowCompactState {38.0_in, -46.0_deg};
  constexpr static ManipulatorSubsystem::ManipulatorState lowReachState {11_in, -5.0_deg};

  ManipulatorSubsystem();

  void Periodic() override;

  /************
   * Elevator *
   ************/
  void setElevatorHeightPercent(double height);
  void setElevatorHeight(units::meter_t height);
  // double getElevatorHeightPercent();
  units::meter_t getElevatorHeight() const;
  bool elevatorAtHeight();
  bool elevatorGoingDown();
  bool elevatorGoingUp();

  void setArmPosition(units::radian_t position);
  void incArmPositon(units::radian_t increment);

  /*******
   * Arm *
   *******/
  units::radian_t getArmPosition() const;

  units::radian_t getArmError() const;
  bool armAtPosition() const;
  bool armIsRaising() const;
  bool armIsLowering() const;

  /*******
   * All *
   *******/
  void setState(const ManipulatorState state);
  frc2::CommandPtr getStateCommand(const ManipulatorState state);
 private:
  /************
   * Elevator *
   ************/
  std::shared_ptr<rmb::LinearPositionFeedbackController> elevatorMotor {
    rmb::asLinear(
      (std::shared_ptr<rmb::AngularPositionFeedbackController>)
      std::make_shared<rmb::SparkMaxPositionController>(
        ManipulatorConstants::Elevator::leader,
        ManipulatorConstants::Elevator::pidConfig,
        ManipulatorConstants::Elevator::feedforward,
        ManipulatorConstants::Elevator::range,
        ManipulatorConstants::Elevator::profileConfig,
        ManipulatorConstants::Elevator::feedbackConfig,
        std::initializer_list<const rmb::SparkMaxPositionController::MotorConfig>{
          ManipulatorConstants::Elevator::follower}
      ),
      ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad
    )
  };
    
  const units::meter_t minElevatorHeight = ManipulatorConstants::Elevator::range.minPosition * (ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad);
  const units::meter_t maxElevatorHeight = ManipulatorConstants::Elevator::range.maxPosition * (ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad);

  /*******
   * Arm *
   *******/
  units::radian_t calculateArmMinPose() const;
  units::radian_t calculateArmMaxPose() const;

  std::shared_ptr<rmb::AngularPositionFeedbackController> armMotor{
    std::make_shared<rmb::SparkMaxPositionController>(
      ManipulatorConstants::Arm::motorConfig,
      ManipulatorConstants::Arm::pidConfig,
      ManipulatorConstants::Arm::feedforward,
      ManipulatorConstants::Arm::range,
      ManipulatorConstants::Arm::profileConfig,
      ManipulatorConstants::Arm::feedbackConfig
    )
  };

  units::radian_t targetArmPose;
};