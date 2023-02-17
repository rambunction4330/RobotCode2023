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

  constexpr static ManipulatorState compactState {11_in, 90.0_deg};
  constexpr static ManipulatorState cubeHighState {38.0_in, 10.0_deg};
  constexpr static ManipulatorState cubeMidState {11_in, 45_deg};
  constexpr static ManipulatorState cubePickupState {38_in, -45_deg};
  constexpr static ManipulatorState coneMidState {11_in, 45_deg};
  constexpr static ManipulatorState conePickupState {11_in, -7.5_deg};
  constexpr static ManipulatorState substationState {38_in, 0.0_deg};

  ManipulatorSubsystem();

  void Periodic() override;

  /************
   * Elevator *
   ************/
  void setElevatorHeight(units::meter_t height);
  void setElevatorHeightPercent(double height);
  units::meter_t getElevatorHeight() const;
  units::meter_t getElevatorError() const;
  bool elevatorAtHeight();
  bool elevatorIsRaising();
  bool elevatorIsLowering();

  /*******
   * Arm *
   *******/

  void setArmAngle(units::radian_t angle);
  void incArmAngle(units::radian_t increment);
  units::radian_t getArmAngle() const;
  units::radian_t getArmError() const;
  bool armAtPosition() const;
  bool armIsRaising() const;
  bool armIsLowering() const;

  /**********
   * States *
   **********/
  void setState(const ManipulatorState state);
  frc2::CommandPtr getStateCommand(const ManipulatorState state);
  frc2::CommandPtr getInstantStateCommand(const ManipulatorState state);

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
        std::initializer_list<
        const rmb::SparkMaxPositionController::MotorConfig>{
          ManipulatorConstants::Elevator::follower
        }
      ),
      ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad
    )
  };
    
  /*******
   * Arm *
   *******/
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

  units::radian_t calculateArmMinPose() const;
  units::radian_t calculateArmMaxPose() const;
};