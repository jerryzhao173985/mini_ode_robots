/* mini_ode_robots — bodysensors.h
 *
 * Proprioceptive Sensors that read a Primitive's body state. Attach with
 * OdeRobot::addSensor(). Grounded in lpzrobots sensors/{speedsensor,axisorientationsensor}.
 *   SpeedSensor          : linear (and optionally angular) velocity of a body
 *   AxisOrientationSensor: the body's local up-axis expressed in world coords
 *                          (== (0,0,1) when upright) — fall/tilt/balance detection
 */
#ifndef MOR_BODYSENSORS_H
#define MOR_BODYSENSORS_H

#include "sensor.h"
#include "primitive.h"
#include "pose_types.h"

namespace mor {

/// linear velocity (3) and optionally angular velocity (+3) of a body, world frame
class SpeedSensor : public Sensor {
public:
  explicit SpeedSensor(Primitive* body, bool withAngular = false)
    : body(body), withAngular(withAngular) {}
  int getSensorNumber() const override { return withAngular ? 6 : 3; }
  int get(double* s, int maxlen) const override {
    int n = 0;
    Pos v = body->getVel();
    if (maxlen>n) s[n++] = v.x(); if (maxlen>n) s[n++] = v.y(); if (maxlen>n) s[n++] = v.z();
    if (withAngular){
      Pos w = body->getAngularVel();
      if (maxlen>n) s[n++] = w.x(); if (maxlen>n) s[n++] = w.y(); if (maxlen>n) s[n++] = w.z();
    }
    return n;
  }
private:
  Primitive* body; bool withAngular;
};

/// the body's local up-axis (+Z) in world coordinates: (0,0,1) upright, tilts as it falls
class AxisOrientationSensor : public Sensor {
public:
  /// @param localAxis which body-local axis to report (default local +Z = "up")
  explicit AxisOrientationSensor(Primitive* body, const Axis& localAxis = Axis(0,0,1))
    : body(body), localAxis(localAxis) {}
  int getSensorNumber() const override { return 3; }
  int get(double* s, int maxlen) const override {
    // transform the local direction by the body's pose rotation (w=0 -> no translation)
    Vec3 a = (localAxis * body->getPose()).vec3();
    int n = 0;
    if (maxlen>n) s[n++] = a.x(); if (maxlen>n) s[n++] = a.y(); if (maxlen>n) s[n++] = a.z();
    return n;
  }
private:
  Primitive* body; Axis localAxis;
};

} // namespace mor
#endif
