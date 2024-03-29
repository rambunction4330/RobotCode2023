#pragma once

#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include <rmb/motorcontrol/AngularPositionController.h>
#include <rmb/motorcontrol/LinearPositionController.h>
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
  constexpr static ManipulatorState cubeHighState {40.0_in, 12.5_deg};
  constexpr static ManipulatorState cubeMidState {11_in, 45_deg};
  constexpr static ManipulatorState cubePickupState {40.0_in, -55_deg};
  constexpr static ManipulatorState coneMidState {11_in, 45_deg};
  constexpr static ManipulatorState coneHighState {40.0_in, 25_deg};
  constexpr static ManipulatorState conePickupState {11_in, -6.0_deg};
  constexpr static ManipulatorState substationState {40.0_in, 7.5_deg};

  ManipulatorSubsystem();

  void Periodic() override;

  /************
   * Elevator *
   ************/
  void setElevatorHeight(units::meter_t height);
  void setElevatorHeightPercent(double height);
  units::meter_t getElevatorHeight() const;
  units::meter_t getTargetElevatorHeight() const;
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
  frc2::CommandPtr manualZeroArmCommand(frc2::CommandPS4Controller& controller);

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
  std::unique_ptr<rmb::LinearPositionController> elevatorMotor {
    rmb::asLinear(
      std::make_unique<rmb::SparkMaxPositionController>(
        rmb::SparkMaxPositionController::CreateInfo {
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
        }
      ),
      ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad
    )
  };
    
  /*******
   * Arm *
   *******/
  std::unique_ptr<rmb::AngularPositionController> armMotor{
    std::make_unique<rmb::SparkMaxPositionController>(
      rmb::SparkMaxPositionController::CreateInfo {
        ManipulatorConstants::Arm::motorConfig,
        ManipulatorConstants::Arm::pidConfig,
        ManipulatorConstants::Arm::feedforward,
        ManipulatorConstants::Arm::range,
        ManipulatorConstants::Arm::profileConfig,
        ManipulatorConstants::Arm::feedbackConfig
      }
    )
  };

  units::radian_t calculateArmMinPose() const;
  units::radian_t calculateArmMaxPose() const;
};
