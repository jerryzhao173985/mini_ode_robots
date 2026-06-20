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

/// Reports the constraint TORQUE (3, and optionally force +3) that a joint applies to
/// its first body — load/contact sensing. Uses the joint feedback already in Joint;
/// enabling it here turns ODE feedback on for that joint.
class ForceTorqueSensor : public Sensor {
public:
  explicit ForceTorqueSensor(Joint* joint, bool withForce = false)
    : joint(joint), withForce(withForce) { joint->setFeedBackMode(true); }
  int getSensorNumber() const override { return withForce ? 6 : 3; }
  int get(double* s, int maxlen) const override {
    int n = 0;
    Pos t1, t2;
    if (joint->getTorqueFeedback(t1, t2)) {
      if(maxlen>n) s[n++]=t1.x(); if(maxlen>n) s[n++]=t1.y(); if(maxlen>n) s[n++]=t1.z();
    } else { for (int k=0;k<3 && maxlen>n;k++) s[n++]=0; }
    if (withForce) {
      Pos f1, f2;
      if (joint->getForceFeedback(f1, f2)) {
        if(maxlen>n) s[n++]=f1.x(); if(maxlen>n) s[n++]=f1.y(); if(maxlen>n) s[n++]=f1.z();
      } else { for (int k=0;k<3 && maxlen>n;k++) s[n++]=0; }
    }
    return n;
  }
private:
  Joint* joint; bool withForce;
};

} // namespace mor
#endif
