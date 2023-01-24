
#pragma once

#include <units/angle.h>
#include <units/base.h>
#include <units/math.h>
#include <units/voltage.h>

#include <wpi/MathExtras.h>

#include "rmb/motorcontrol/feedforward/Feedforward.h"

namespace rmb {

/**
 * Voltage feedforward for an arm.
 * @tparam DistanceUnit Base unit of distance for feedforward inputs.
 **/
class ArmFeedforward : public Feedforward<units::radians> {
public:
  using Distance_t = typename Feedforward<units::radians>::
      Distance_t; /**< @see Feedforward<DistanceUnit>::Distance_t*/
  using VelocityUnit = typename Feedforward<units::radians>::
      VelocityUnit; /**< @see Feedforward<DistanceUnit>::VelocityUnit*/
  using Velocity_t = typename Feedforward<units::radians>::
      Velocity_t; /**< @see Feedforward<DistanceUnit>::Velocity_t*/
  using AccelerationUnit = typename Feedforward<units::radians>::
      AccelerationUnit; /**< @see Feedforward<DistanceUnit>::AccelerationUnit*/
  using Acceleration_t = typename Feedforward<units::radians>::
      Acceleration_t; /**< @see Feedforward<DistanceUnit>::Acceleration_t */

  using KsUnit = typename Feedforward<
      units::radians>::KsUnit; /**< @see Feedforward<DistanceUnit>::KsUnit*/
  using Ks_t = typename Feedforward<
      units::radians>::Ks_t; /**< @see Feedforward<DistanceUnit>::Ks_t*/
  using KvUnit = typename Feedforward<
      units::radians>::KvUnit; /**< @see Feedforward<DistanceUnit>::KvUnit*/
  using Kv_t = typename Feedforward<
      units::radians>::Kv_t; /**< @see Feedforward<DistanceUnit>::Kv_t*/
  using KaUnit = typename Feedforward<
      units::radians>::KaUnit; /**< @see Feedforward<DistanceUnit>::KaUnit*/
  using Ka_t = typename Feedforward<
      units::radians>::Ka_t; /**< @see Feedforward<DistanceUnit>::Ka_t*/

  /**
   * Create an ArmFeedforward
   * @param kS Static gain
   * @param kCos Cosine gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   */
  ArmFeedforward(Ks_t kS, Ks_t kCos, Kv_t kV, Ka_t kA)
      : kS(kS), kCos(kCos), kV(kV), kA(kA){};

  /**
   * Calculates a feedforward voltage at a desired velocity, acceleration,
   * and distance.
   *
   * @param velocity Desired Velocity
   * @param position Position of Motor (Not always useful).
   * @param acceleration Desired Acceleration
   **/
  inline units::volt_t calculate(Velocity_t velocity, Distance_t position,
                                 Acceleration_t acceleration) const override {
    return kS * wpi::sgn(velocity) + kCos * units::math::cos(position) +
           kV * velocity + kA * acceleration;
  }

  /**
   * Calculates the minimum achievable velocity of a component.
   *
   * @param maxVoltage max voltage that can be applied
   * @param acceleration acceleration that this velocity is achived at
   * @param position position that this veloocity is achived at
   *
   * @return Maximum achivable velocity.
   **/
  inline Velocity_t maxAchievableVelocity(units::volt_t maxVoltage,
                                          Acceleration_t acceleration,
                                          Distance_t position) const override {
    return (maxVoltage - kS - kCos * units::math::cos(position) -
            kA * acceleration) /
           kV;
  }

  /**
   * Calculates the minimum achievable velocity of a component.
   *
   * @param maxVoltage max voltage that can be applied
   * @param acceleration acceleration that this velocity is achived at
   * @param position position that this veloocity is achived at
   *
   * @return Minimum achivable velocity.
   **/
  inline Velocity_t minAchievableVelocity(units::volt_t maxVoltage,
                                          Acceleration_t acceleration,
                                          Distance_t position) const override {
    return (-maxVoltage + kS - kCos * units::math::cos(position) -
            kA * acceleration) /
           kV;
  }

  /**
   * Calculates the maximum achievable accceleration of a component.
   *
   * @param maxVoltage max voltage that can be applied
   * @param velocity velocity that this acceleration is achived at
   * @param position position that this acceleration is achived at
   *
   * @return Maximum achivable acceleration.
   **/
  inline Acceleration_t
  maxAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity,
                            Distance_t position) const override {
    return (maxVoltage - kS * wpi::sgn(velocity) -
            kCos * units::math::cos(position) - kV * velocity) /
           kA;
  }

  /**
   * Calculates the minimum achievable accceleration of a component.
   *
   * @param maxVoltage max voltage that can be applied
   * @param velocity velocity that this acceleration is achived at
   * @param position position that this acceleration is achived at
   *
   * @return Minimum achivable acceleration.
   **/
  inline Acceleration_t
  minAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity,
                            Distance_t position) const override {
    return maxAchievableAcceleration(-maxVoltage, velocity, position);
  }

  /**
   * Return the velocity gain of feed forward. This is the value that velocity
   * is multiplied by when calculating voltage. This is useful when adding
   * feedforwads to the PID loops of motor controllers.
   *
   * @return Velocity gain.
   **/
  inline Kv_t getVelocityGain() const override { return kV; }

  /**
   * Return the acceleration gain of feed forward. This is the value that
   * acceleration is multiplied by when calculating voltage. This is useful
   * when adding feedforwads to the PID loops of motor controllers.
   *
   * @return Acceleration gain.
   **/
  inline Ka_t getAcclerationGain() const override { return kA; }

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
  inline units::volt_t
  calculateStatic(Velocity_t velocity,
                  Distance_t position = Distance_t(0)) const override {
    return kS * wpi::sgn(velocity) + kCos * units::math::cos(position);
  }

private:
  Ks_t kS, kCos; /* Static gain. */
  Kv_t kV;       /* Velocity gain. */
  Ka_t kA;       /* Acelrration gain. */
};

} // namespace rmb
