#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/LinearVelocityController.h"
#include "rmb/motorcontrol/LinearPositionController.h"

namespace rmb {

//---------------------------
// AngularVelocityController
//---------------------------

class AngularAsLinearVelocityController: public LinearVelocityController {
public:

  AngularAsLinearVelocityController(std::shared_ptr<AngularVelocityController> angularController, 
                                    MotorControlConversions::ConversionUnit_t conversionFactor) :
                                    angular(angularController), conversion(conversionFactor) {}
  
  void setVelocity(units::meters_per_second_t velocity) { angular->setVelocity(velocity / conversion); }
  units::meters_per_second_t getTargetVelocity() const { return angular->getTargetVelocity() * conversion; }
  units::meters_per_second_t getMaxVelocity() const { return angular->getMaxVelocity() * conversion; }
  void setPower(double power) { angular->setPower(power); }
  void disable() { angular->disable(); }
  void stop() { angular->stop(); }

private:
  std::shared_ptr<AngularVelocityController> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<LinearVelocityController> asLinear(std::shared_ptr<AngularVelocityController> angularController,
                                                   MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<AngularAsLinearVelocityController>(angularController, conversion);
}

//---------------------------
// AngularPositionController
//---------------------------

class AngularAsLinearPositionController : public LinearPositionController {
public:

  AngularAsLinearPositionController(std::shared_ptr<AngularPositionController> angularController, 
                                    MotorControlConversions::ConversionUnit_t conversionFactor) :
                                    angular(angularController), conversion(conversionFactor) {}

  void setPosition(units::meter_t position) { angular->setPosition(position / conversion); }
  units::meter_t getTargetPosition() const { return angular->getTargetPosition() * conversion; }
  units::meter_t getMinPosition() const { return angular->getMinPosition() * conversion; }
  units::meter_t getMaxPosition() const { return angular->getMaxPosition() * conversion; }
  void disable() { angular->disable(); }
  void stop() { angular->stop(); }

private:
  std::shared_ptr<AngularPositionController> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<LinearPositionController> asLinear(std::shared_ptr<AngularPositionController> angularController, 
                                                   MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<AngularAsLinearPositionController>(angularController, conversion);
}

//--------------------------
// LinearVelocityController
//--------------------------

class LinearAsAngularVelocityController: public AngularVelocityController {
 public:

  LinearAsAngularVelocityController(std::shared_ptr<LinearVelocityController> linearController, 
                                    MotorControlConversions::ConversionUnit_t conversionFactor) :
                                    linear(linearController), conversion(conversionFactor) {}

  void setVelocity(units::radians_per_second_t velocity) { linear->setVelocity(velocity * conversion); }
  units::radians_per_second_t getTargetVelocity() const { return linear->getTargetVelocity() / conversion; }
  units::radians_per_second_t getMaxVelocity() const { return linear->getMaxVelocity() / conversion; }
  void setPower(double power) { linear->setPower(power); }
  void disable() { linear->disable(); }
  void stop() { linear->stop(); }

 private:
  std::shared_ptr<LinearVelocityController> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<AngularVelocityController> asAngular(std::shared_ptr<LinearVelocityController> linearController,
                                                     MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<LinearAsAngularVelocityController>(linearController, conversion);
}

//--------------------------
// LinearPositionController
//--------------------------

class LinearAsAngularPositionController : public AngularPositionController {
public:

  LinearAsAngularPositionController(std::shared_ptr<LinearPositionController> linearController, 
                                    MotorControlConversions::ConversionUnit_t conversionFactor) :
                                    linear(linearController), conversion(conversionFactor) {}

  void setPosition(units::radian_t position) { linear->setPosition(position * conversion); }
  units::radian_t getTargetPosition() const { return linear->getTargetPosition() / conversion; }
  units::radian_t getMinPosition() const { return linear->getMinPosition() / conversion; }
  units::radian_t getMaxPosition() const { return linear->getMaxPosition() / conversion; }
  void disable() { linear->disable(); }
  void stop() { linear->stop(); }

private:
  std::shared_ptr<LinearPositionController> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

std::shared_ptr<AngularPositionController> asAngular(std::shared_ptr<LinearPositionController> linearController, 
                                                     MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_shared<LinearAsAngularPositionController>(linearController, conversion);
}
} // namespace rmb