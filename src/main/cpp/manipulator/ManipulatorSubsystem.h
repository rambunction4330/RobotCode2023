
#pragma once

#include <frc2/command/SubsystemBase.h>

#include <memory>

#include <rmb/motorcontrol/feedback/AngularPositionFeedbackController.h>
#include <rmb/motorcontrol/feedback/LinearPositionFeedbackController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

#include "manipulator/ManipulatorConstants.h"

class ManipulatorSubsystem : public frc2::SubsystemBase {
  public:
    void Periodic() override;

    ManipulatorSubsystem();

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

    /************
     * Arm      *
     ************/
    units::radian_t getArmPosition() const;

    units::radian_t getArmError() const;
    bool armAtPosition() const;
    bool armIsRaising() const;
    bool armIsLowering() const;

    /************
     * Claw     *
     ************/
    void setClawClosed(bool isClosed);
    void toggleClaw();
    bool getClawClosed() const;

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
                        ManipulatorConstants::Elevator::follower}),
            ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad)};
    
    const units::meter_t minElevatorHeight = ManipulatorConstants::Elevator::range.minPosition * (ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad);
    const units::meter_t maxElevatorHeight = ManipulatorConstants::Elevator::range.maxPosition * (ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad);

    /************
     * Arm      *
     ************/
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

    /************
     * Claw      *
     ************/

    std::shared_ptr<rmb::AngularPositionFeedbackController> clawMotor{
      std::make_shared<rmb::SparkMaxPositionController>(
        ManipulatorConstants::Claw::motorConfig,
        ManipulatorConstants::Claw::pidConfig,
        ManipulatorConstants::Claw::feedforward,
        ManipulatorConstants::Claw::range,
        ManipulatorConstants::Claw::profileConfig,
        ManipulatorConstants::Claw::feedbackConfig
      )
    }; 

  bool clawClosed = false;
};