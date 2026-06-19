/* mini_ode_robots — sensor.h
 *
 * The attachable Sensor interface (lpzrobots' core extensibility idea, restored).
 * A Sensor produces one or more scalar readings each step. Attach any number to a
 * robot with OdeRobot::addSensor(); the robot's getSensors() then appends their
 * values automatically — so a new robot gets ray/joint/… sensing for free without
 * rewriting its I/O. Concrete sensors: RaySensor (raysensor.h), JointSensor (below).
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
