
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

    ManipulatorSubsystem() = default;

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

  private:
    /************
     * Elevator *
     ************/
    std::shared_ptr<rmb::LinearPositionFeedbackController> elevatorMotor{
        rmb::asLinear(
            (std::shared_ptr<rmb::AngularPositionFeedbackController>)
                std::make_shared<rmb::SparkMaxPositionController>(
                    ManipulatorConstants::Elevator::leader,
                    ManipulatorConstants::Elevator::pidConfig,
                    ManipulatorConstants::Elevator::range,
                    ManipulatorConstants::Elevator::profileConfig,
                    ManipulatorConstants::Elevator::feedbackConfig,
                    std::initializer_list<
                        const rmb::SparkMaxPositionController::MotorConfig>{
                        ManipulatorConstants::Elevator::follower}),
            ManipulatorConstants::Elevator::sproketDiameter / 2.0_rad)};

    /************
     * Arm      *
     ************/
    // TODO: Dynamic Arm Range
    units::radian_t calculateArmMinPose() const;
    units::radian_t calculateArmMaxPose() const;

    std::shared_ptr<rmb::AngularPositionFeedbackController> armMotor{
        std::make_shared<rmb::SparkMaxPositionController>(
            ManipulatorConstants::Arm::motorConfig,
            ManipulatorConstants::Arm::pidConfig,
            ManipulatorConstants::Arm::range,
            ManipulatorConstants::Arm::profileConfig,
            ManipulatorConstants::Arm::feedbackConfig)};
};