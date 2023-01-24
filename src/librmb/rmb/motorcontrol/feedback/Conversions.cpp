#include "rmb/motorcontrol/feedback/AngularEncoder.h"
#include "rmb/motorcontrol/feedback/AngularVelocityFeedbackController.h"
#include "rmb/motorcontrol/feedback/AngularPositionFeedbackController.h"

#include "rmb/motorcontrol/feedback/LinearEncoder.h"
#include "rmb/motorcontrol/feedback/LinearVelocityFeedbackController.h"
#include "rmb/motorcontrol/feedback/LinearPositionFeedbackController.h"

namespace rmb {

//----------------
// AngularEncoder
//----------------

class AngularAsLinearEncoder : public LinearEncoder {
public:

  AngularAsLinearEncoder(std::shared_ptr<AngularEncoder> angularEncoder, MotorControlConversions::ConversionUnit_t conversionFactor) :
                         angular(std::move(angularEncoder)), conversion(conversionFactor) {}

  units::meters_per_second_t getVelocity() const { return angular->getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular->getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular->zeroPosition(offset / conversion); }

private:
  std::shared_ptr<AngularEncoder> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<LinearEncoder> asLinear(std::shared_ptr<AngularEncoder> angularEncoder, 
                                        MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<AngularAsLinearEncoder>(angularEncoder, conversion);
}

//-----------------------------------
// AngularVelocityFeedbackController
//-----------------------------------

class AngularAsLinearVelocityFeedbackController : public LinearVelocityFeedbackController {
public:

  AngularAsLinearVelocityFeedbackController(std::shared_ptr<AngularVelocityFeedbackController> angularController, 
                                            MotorControlConversions::ConversionUnit_t conversionFactor) :
                                            angular(angularController), conversion(conversionFactor) {}

  // Controller Methods
  void setVelocity(units::meters_per_second_t velocity) { angular->setVelocity(velocity / conversion); }
  units::meters_per_second_t getTargetVelocity() const { return angular->getTargetVelocity() * conversion; }
  units::meters_per_second_t getMaxVelocity() const { return angular->getMaxVelocity() * conversion; }
  void setPower(double power) { angular->setPower(power); }
  void disable() { angular->disable(); }
  void stop() { angular->stop(); }

  // Encoder Methods
  units::meters_per_second_t getVelocity() const { return angular->getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular->getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular->zeroPosition(offset / conversion); }

  // Feedback Methods
  units::meters_per_second_t getTolerance() const { return angular->getTolerance() * conversion; }

private:
  std::shared_ptr<AngularVelocityFeedbackController> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<LinearVelocityFeedbackController> asLinear(std::shared_ptr<AngularVelocityFeedbackController> angularController, 
                                                           MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<AngularAsLinearVelocityFeedbackController>(angularController, conversion);
}

//-----------------------------------
// AngularPositionFeedbackController
//-----------------------------------

class AngularAsLinearPositionFeedbackController : public LinearPositionFeedbackController {
public:

  AngularAsLinearPositionFeedbackController(std::shared_ptr<AngularPositionFeedbackController> angularController, 
                                            MotorControlConversions::ConversionUnit_t conversionFactor) :
                                            angular(angularController), conversion(conversionFactor) {}
  
  // Controller Methods
  void setPosition(units::meter_t position) { angular->setPosition(position / conversion); }
  units::meter_t getTargetPosition() const { return angular->getTargetPosition() * conversion; }
  units::meter_t getMinPosition() const { return angular->getMinPosition() * conversion; }
  units::meter_t getMaxPosition() const { return angular->getMaxPosition() * conversion; }
  void disable() { angular->disable(); }
  void stop() { angular->stop(); }

  // Encoder Methods
  units::meters_per_second_t getVelocity() const { return angular->getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular->getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular->zeroPosition(offset / conversion); }

  // Feedback Methods
  units::meter_t getTolerance() const { return angular->getTolerance() * conversion; }

private:
  std::shared_ptr<AngularPositionFeedbackController> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<LinearPositionFeedbackController> asLinear(std::shared_ptr<AngularPositionFeedbackController> angularController,
                                                           MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<AngularAsLinearPositionFeedbackController>(angularController, conversion);
}

//---------------
// LinearEncoder
//---------------

class LinearAsAngularEncoder : public AngularEncoder {
public:

  LinearAsAngularEncoder(std::shared_ptr<LinearEncoder> linearEncoder, 
                         MotorControlConversions::ConversionUnit_t conversionFactor) :
                         linear(linearEncoder), conversion(conversionFactor) {}

  units::radians_per_second_t getVelocity() const { return linear->getVelocity() / conversion; }
  units::radian_t getPosition() const { return linear->getPosition() / conversion; }
  void zeroPosition(units::radian_t offset = 0_rad) { linear->zeroPosition(offset * conversion); }

private:
  std::shared_ptr<LinearEncoder> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<AngularEncoder> asAngular(std::shared_ptr<LinearEncoder> linearEncoder, 
                                          MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<LinearAsAngularEncoder>(linearEncoder, conversion);
}

//----------------------------------
// LinearVelocityFeedbackController
//----------------------------------

class LinearAsAngularVelocityFeedbackController : public AngularVelocityFeedbackController {
public:

  LinearAsAngularVelocityFeedbackController(std::shared_ptr<LinearVelocityFeedbackController> linearController, 
                                            MotorControlConversions::ConversionUnit_t conversionFactor) :
                                            linear(linearController), conversion(conversionFactor) {}

  // Controller Methods
  void setVelocity(units::radians_per_second_t velocity) { linear->setVelocity(velocity * conversion); }
  units::radians_per_second_t getTargetVelocity() const { return linear->getTargetVelocity() / conversion; }
  units::radians_per_second_t getMaxVelocity() const { return linear->getMaxVelocity() / conversion; }
  void setPower(double power) { linear->setPower(power); }
  void disable() { linear->disable(); }
  void stop() { linear->stop(); }

  // Encoder Methods
  units::radians_per_second_t getVelocity() const { return linear->getVelocity() / conversion; }
  units::radian_t getPosition() const { return linear->getPosition() / conversion; }
  void zeroPosition(units::radian_t offset = 0_rad) { linear->zeroPosition(offset * conversion); }

  // Feedback Methods
  units::radians_per_second_t getTolerance() const { return linear->getTolerance() / conversion; }

private:
  std::shared_ptr<LinearVelocityFeedbackController> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<AngularVelocityFeedbackController> asAngular(std::shared_ptr<LinearVelocityFeedbackController> linearController,
                                                            MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<LinearAsAngularVelocityFeedbackController>(linearController, conversion);
}

//----------------------------------
// LinearPositionFeedbackController
//----------------------------------

class LinearAsAngularPositionFeedbackController : public AngularPositionFeedbackController {
public:

  LinearAsAngularPositionFeedbackController(std::shared_ptr<LinearPositionFeedbackController> linearController, 
                                            MotorControlConversions::ConversionUnit_t conversionFactor) :
                                            linear(linearController), conversion(conversionFactor) {}

  // Controller Methods
  void setPosition(units::radian_t position) { linear->setPosition(position * conversion); }
  units::radian_t getTargetPosition() const { return linear->getTargetPosition() / conversion; }
  units::radian_t getMinPosition() const { return linear->getMinPosition() / conversion; }
  units::radian_t getMaxPosition() const { return linear->getMaxPosition() / conversion; }
  void disable() { linear->disable(); }
  void stop() { linear->stop(); }

  // Encoder Methods
  units::radians_per_second_t getVelocity() const { return linear->getVelocity() / conversion; }
  units::radian_t getPosition() const { return linear->getPosition() / conversion; }
  void zeroPosition(units::radian_t offset = 0_rad) { linear->zeroPosition(offset * conversion); }

  // Feedback Methods
  units::radian_t getTolerance() const { return linear->getTolerance() / conversion; }

private:
  std::shared_ptr<LinearPositionFeedbackController> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<AngularPositionFeedbackController> asAngular(std::shared_ptr<LinearPositionFeedbackController> linearController,
                                                             MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<LinearAsAngularPositionFeedbackController>(linearController, conversion);
}
} // namespace rmb