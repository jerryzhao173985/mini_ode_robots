/* mini_ode_robots — sensor.h
 *
 * The attachable SENSOR interface (lpzrobots' core extensibility idea, restored).
 * A Sensor produces one or more scalar readings each step. Attach any number with
 * OdeRobot::addSensor() — the robot OWNS it and frees it in cleanup(); the robot's
 * getSensors() then appends each sensor's values after its own channels, so a new
 * robot gets ray/joint/contact/… sensing without rewriting its I/O.
 *
 * Per-step contract: Simulation::run calls the robot's sense() (which calls each
 * attached sensor's sense(global)) BEFORE the controller reads getSensors() -> get().
 * So sense() should LATCH/compute the reading; get(buf,maxlen) just copies out, writes
 * at most maxlen values, and returns the count written.
 *
 * Concrete sensors: RaySensor (raysensor.h); JointSensor + ForceTorqueSensor
 * (jointsensor.h); SpeedSensor + AxisOrientationSensor (bodysensors.h); ContactSensor
 * (contactsensor.h).
 */
#ifndef MOR_SENSOR_H
#define MOR_SENSOR_H

#include "globaldata.h"

namespace mor {

class Primitive;

class Sensor {
public:
  virtual ~Sensor() = default;
  /// number of scalar values this sensor contributes
  virtual int  getSensorNumber() const = 0;
  /// active sensing for this step (called before get(); cache the reading here)
  virtual void sense(const GlobalData& global) { (void)global; }
  /// write up to `maxlen` values into `sensors`; return how many were written
  virtual int  get(double* sensors, int maxlen) const = 0;
};

} // namespace mor
#endif
