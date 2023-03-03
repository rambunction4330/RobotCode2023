#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rmb/motorcontrol/feedback/AngularPositionFeedbackController.h>
#include <rmb/motorcontrol/feedback/LinearPositionFeedbackController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxLowLevel.h"

class ClawSubsystem : public frc2::SubsystemBase {
  public:
    void Periodic() override;

    ClawSubsystem();

    void setClawClosed(bool isClosed, double assistance = 0.0);
    bool getClawClosed() const;

    frc2::CommandPtr getClosedCommand(bool setClosed, double assistance = 0.0);

  private:
    ctre::phoenix::motorcontrol::can::WPI_TalonFX clawMotor {41};
    bool clawClosed = false;

    double currentAssistance = 0.0;
};
