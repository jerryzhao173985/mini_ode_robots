/* mini_ode_robots — jointsensor.h
 * Proprioceptive Sensor: reports a OneAxisJoint's angle (and optionally its rate).
 * Attach to a robot with addSensor(new JointSensor(hinge)) to expose joint feedback
 * to the controller without hand-writing getSensorsIntern.
 */
#ifndef MOR_JOINTSENSOR_H
#define MOR_JOINTSENSOR_H

#include "sensor.h"
#include "joint.h"

namespace mor {

class JointSensor : public Sensor {
public:
  explicit JointSensor(OneAxisJoint* joint, bool withRate = false)
    : joint(joint), withRate(withRate) {}
  int getSensorNumber() const override { return withRate ? 2 : 1; }
  int get(double* s, int maxlen) const override {
    int n = 0;
    if (maxlen > n) s[n++] = joint->getPosition1();
    if (withRate && maxlen > n) s[n++] = joint->getPosition1Rate();
    return n;
  }
private:
  OneAxisJoint* joint;
  bool withRate;
};

} // namespace mor
#endif
