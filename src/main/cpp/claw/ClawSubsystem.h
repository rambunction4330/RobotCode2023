
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rmb/motorcontrol/feedback/AngularPositionFeedbackController.h>
#include <rmb/motorcontrol/feedback/LinearPositionFeedbackController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

#include "claw/ClawConstants.h"

class ClawSubsystem : public frc2::SubsystemBase {
  public:
    void Periodic() override;

    ClawSubsystem();

    void setClawClosed(bool isClosed);
    void toggleClaw();
    bool getClawClosed() const;

    frc2::CommandPtr getClosedCommand(bool setClosed);

  private:
    std::shared_ptr<rmb::AngularPositionFeedbackController> clawMotor{
      std::make_shared<rmb::SparkMaxPositionController>(
        ClawConstants::motorConfig,
        ClawConstants::pidConfig,
        ClawConstants::feedforward,
        ClawConstants::range,
        ClawConstants::profileConfig,
        ClawConstants::feedbackConfig
      )
    }; 

    bool clawClosed = false;
};