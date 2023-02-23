#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rmb/motorcontrol/feedback/AngularPositionFeedbackController.h>
#include <rmb/motorcontrol/feedback/LinearPositionFeedbackController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxLowLevel.h"

class ClawSubsystem : public frc2::SubsystemBase {
  public:
    void Periodic() override;

    ClawSubsystem();

    void setClawClosed(bool isClosed);
    bool getClawClosed() const;

    frc2::CommandPtr getClosedCommand(bool setClosed);

  private:
    rev::CANSparkMax clawMotor {41, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    bool clawClosed = false;
};
