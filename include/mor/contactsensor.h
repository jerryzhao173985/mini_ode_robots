/* mini_ode_robots — contactsensor.h
 *
 * Binary touch sensor (lpzrobots sensors/contactsensor): 1.0 if the owner primitive
 * collided this step, else 0.0 — e.g. foot-ground contact for walking gaits. It
 * installs a collision callback on the owner's Substance that records the touch and
 * returns 1 (use default surface), so the part both COLLIDES physically and SENSES.
 */
#ifndef MOR_CONTACTSENSOR_H
#define MOR_CONTACTSENSOR_H

#include "sensor.h"
#include "substance.h"
#include "primitive.h"

namespace mor {

class ContactSensor : public Sensor {
public:
  explicit ContactSensor(Primitive* owner) : owner(owner) {
    owner->substance.setCollisionCallback(&ContactSensor::cb, this);
  }
  ~ContactSensor() override { if (owner) owner->substance.setCollisionCallback(nullptr, nullptr); }

  int  getSensorNumber() const override { return 1; }
  void sense(const GlobalData& g) override {
    if (g.sim_step != lasttime) { value = touched ? 1.0 : 0.0; touched = false; }
    lasttime = g.sim_step;
  }
  int  get(double* s, int maxlen) const override { if (maxlen<1) return 0; s[0]=value; return 1; }

  void registerTouch() { touched = true; }

private:
  static int cb(dSurfaceParameters&, GlobalData&, void* ud, dContact*, int,
                dGeomID, dGeomID, const Substance&, const Substance&) {
    static_cast<ContactSensor*>(ud)->registerTouch();
    return 1;   // 1 = use default surface params: the contact still happens physically
  }
  Primitive* owner;
  bool  touched  = false;
  double value   = 0.0;
  long  lasttime = -1;
};

} // namespace mor
#endif
