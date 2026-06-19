/* mini_ode_robots — motor.h
 *
 * The attachable Motor interface (mirror of Sensor). A Motor consumes one or more
 * command values and applies them to the physics each step. Attach with
 * OdeRobot::addMotor(); the robot's setMotors() routes the matching slice of the
 * command vector to it, and doInternalStuff() calls act() every step.
 * Concrete motor: OneAxisServo / OneAxisServoVel (servo.h).
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
