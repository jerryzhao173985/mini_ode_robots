/* mini_ode_robots — raysensor.cpp
 * Extracted from lpzrobots sensors/raysensor.cpp (visuals removed).
 */
#include "mor/raysensor.h"
#include "mor/substance.h"
#include <algorithm>

namespace mor {

// Fires on every ray collision: record the hit distance (contact depth) and
// return 0 so the engine creates NO contact joint (the ray is a pure sensor).
static int rayCollCallback(dSurfaceParameters&, GlobalData& g, void* userdata,
                           dContact* contacts, int numContacts,
                           dGeomID, dGeomID, const Substance&, const Substance&){
  (void)g; (void)numContacts;
  RaySensor* s = static_cast<RaySensor*>(userdata);
  s->registerHit(contacts[0].geom.depth);   // depth = distance along the ray to the hit
  return 0;
}

RaySensor::RaySensor(double range)
  : range(range), len(range), detection(range), lasttime(-1),
    ray(nullptr), transform(nullptr), initialised(false) {}

RaySensor::~RaySensor(){
  if (transform) delete transform;   // Transform owns (and deletes) the child ray
}

void RaySensor::init(const OdeHandle& odeHandle, Primitive* own, const Pose& pose){
  len = range; detection = range;
  ray = new Ray(range);
  transform = new Transform(own, ray, pose);
  transform->init(odeHandle, 0, Primitive::Geom);
  // route the ray's collisions to this sensor; rv=0 means "no physical contact"
  transform->substance.setCollisionCallback(rayCollCallback, this);
  initialised = true;
}

void RaySensor::sense(const GlobalData& g){
  if (g.sim_step != lasttime){
    len = std::max(0.0, std::min(detection, range));
    detection = range;
  }
  lasttime = g.sim_step;
}

} // namespace mor
