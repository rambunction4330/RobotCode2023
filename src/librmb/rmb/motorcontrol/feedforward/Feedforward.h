
#pragma once

#include <units/base.h>
#include <units/time.h>
#include <units/voltage.h>

namespace rmb {

/**
 * Generalized interface for voltage feedforwards.
 * @tparam DistanceUnit Base unit of distance for feedforward inputs.
 **/
template <typename DistanceUnit> 
class Feedforward {
public:
  using Distance_t = units::unit_t<DistanceUnit>; /**< User specified distance type*/
  using VelocityUnit = units::compound_unit<DistanceUnit, units::inverse<units::seconds>>; /**< DistanceUnit / Second*/
  using Velocity_t = units::unit_t<VelocityUnit>; /**< Type declaration for FeedForward<DistanceUnit>::VelocityUnit*/
  using AccelerationUnit = units::compound_unit<VelocityUnit, units::inverse<units::seconds>>; /**< VelocityUnit / Second*/
  using Acceleration_t = units::unit_t<AccelerationUnit>; /**< Type declaration for FeedForward<DistanceUnit>::AccelerationUnit*/

  using KsUnit = units::volts; /**< Static gain is in volts */
  using Ks_t = units::unit_t<KsUnit>; /**< Type declaration for Feedforward<DistanceUnit>::KsUnit*/
  using KvUnit = units::compound_unit<units::volts, units::inverse<VelocityUnit>>; /**< Velocity gain unit is Volts / VelocityUnit*/
  using Kv_t = units::unit_t<KvUnit>; /**< Type declaration for FeedForward<DistanceUnit>::KvUnit*/
  using KaUnit = units::compound_unit<units::volts, units::inverse<AccelerationUnit>>; /**< Acceleration gain unit is volts / AcclerationUnit*/
  using Ka_t = units::unit_t<KaUnit>; /**< Type declaration for FeedForward<DistanceUnit>::KaUnit*/

  /**
   * Calculates a feedforward voltage at a desired velocity, acceleration,
   * and distance.
   *
   * @param velocity Desired Velocity
   * @param distance Position of Motor (Not always useful).
   * @param acceleration Desired Acceleration
   **/
  virtual units::volt_t
  calculate(Velocity_t velocity, Distance_t distance = Distance_t(0.0),
            Acceleration_t acceleration = Acceleration_t(0.0)) const = 0;

  /**
   * Calculates the minimum achievable velocity of a component.
   *
   * @param maxVoltage max voltage that can be applied
   * @param acceleration acceleration that this velocity is achived at
   * @param position position that this veloocity is achived at
   *
   * @return Maximum achivable velocity.
   **/
  virtual Velocity_t maxAchievableVelocity(units::volt_t maxVoltage,
                                                     Acceleration_t acceleration,
                                                     Distance_t position) const = 0;

  /**
   * Calculates the minimum achievable velocity of a component.
   *
   * @param maxVoltage max voltage that can be applied
   * @param acceleration acceleration that this velocity is achived at
   * @param position position that this veloocity is achived at
   *
   * @return Minimum achivable velocity.
   **/
  virtual Velocity_t minAchievableVelocity(units::volt_t maxVoltage,
                                                     Acceleration_t acceleration,
                                                     Distance_t position) const = 0;
  /**
   * Calculates the maximum achievable accceleration of a component.
   *
   * @param maxVoltage max voltage that can be applied
   * @param velocity velocity that this acceleration is achived at
   * @param position position that this acceleration is achived at
   *
   * @return Maximum achivable acceleration.
   **/
  virtual Acceleration_t
  maxAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity,
                            Distance_t position) const = 0;

  /**
   * Calculates the minimum achievable accceleration of a component.
   *
   * @param maxVoltage max voltage that can be applied
   * @param velocity velocity that this acceleration is achived at
   * @param position position that this acceleration is achived at
   *
   * @return Minimum achivable acceleration.
   **/
  virtual Acceleration_t
  minAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity,
                            Distance_t position) const = 0;

  /**
   * Return the velocity gain of feed forward. This is the value that velocity
   * is multiplied by when calculating voltage. This is useful when adding
   * feedforwads to the PID loops of motor controllers.
   *
   * @return Velocity gain.
   **/
  virtual Kv_t getVelocityGain() const = 0;

  /**
   * Return the acceleration gain of feed forward. This is the value that
   * acceleration is multiplied by when calculating voltage. This is useful
   * when adding feedforwads to the PID loops of motor controllers.
   *
   * @return Acceleration gain.
   **/
  virtual Ka_t getAcclerationGain() const = 0;

  /**
   * Calculates the static gain of the feedforward at a given position. This
   * is the value added on tot he end of the feedforward calculation. A
   * velocity term is included only to determine the direction of movment.
   * This is useful when adding feedforwads to the PID loops of motor
   *controllers.
   *
   * @param velocity term only to determine the direction of movment (positive
   *or negetive).
   * @param position positon at which the static gain is calculated.
   *
   * @return Static gain.
   **/
  virtual units::volt_t
  calculateStatic(Velocity_t velocity,
                  Distance_t position = Distance_t(0)) const = 0;
};
} // namespace rmb
