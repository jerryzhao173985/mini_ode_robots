/* mini_ode_robots — motor.h
 *
 * The attachable ACTUATOR interface. A Motor consumes command values and applies them
 * to the physics each step. Attach with OdeRobot::addMotor() — the robot OWNS it and
 * frees it in cleanup(); setMotors() routes the matching command slice to set(), and
 * doInternalStuff() calls act() each step.
 *
 * Actuator taxonomy (two kinds, distinguished by OWNERSHIP, not capability):
 *   - SERVOS  (servo.h: OneAxisServo, OneAxisServoVel, TwoAxisServoVel) are VALUE types
 *     a robot holds directly (std::vector<...>) and drives via set()/getPos() inside
 *     doInternalStuffIntern. Not polymorphic — the common, lightweight joint case.
 *   - MOTORS  (this interface) are POLYMORPHIC, heap-attached actuators for composition.
 *     The concrete one is AngularMotor (angularmotor.h): an ODE dAMotor that actively
 *     drives a ball joint / arbitrary axes — which a servo-on-one-joint cannot.
 *
 * Lifecycle contract: a Motor's ODE handles must not outlive the world; guard any
 * ODE-destroy in the dtor behind odeIsAlive() (see AngularMotor).
 */
#ifndef MOR_MOTOR_H
#define MOR_MOTOR_H

#include "globaldata.h"

namespace mor {

class Motor {
public:
  virtual ~Motor() = default;
  /// number of command values this motor consumes
  virtual int  getMotorNumber() const = 0;
  /// store commands (read up to `len`); applied in act()
  virtual void set(const double* motors, int len) = 0;
  /// apply the stored command to the physics for this step
  virtual void act(const GlobalData& global) { (void)global; }
};

} // namespace mor
#endif
