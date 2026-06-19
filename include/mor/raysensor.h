/* mini_ode_robots — raysensor.h
 *
 * A distance/IR sensor built on the ODE Ray geom, extracted from lpzrobots
 * sensors/raysensor. A Ray is rigidly attached to an owner body via a Transform
 * and points along its local +Z. On each collision the Substance callback reads
 * the contact depth (distance to the hit) and returns 0 so NO physical contact
 * joint is made — the ray is a pure sensor. sense() latches the nearest hit per
 * step; get() returns the measured distance (range if nothing was hit).
 *
 * Exercises three engine features at once: Ray, Transform (composite geom),
 * and the per-Substance collision callback.
 *
 * NOTE: the ray reports the nearest hit among the geoms in its space (ODE auto-
 * excludes geoms on the ray's OWN body, so it never hits its owner — but it CAN hit
 * sibling robot parts that are different bodies in the same space). To make it ignore
 * a specific sibling: `odeHandle.addIgnoredPair(raySensor.getGeom(), siblingGeom)`.
 * To ignore a whole robot, build the robot in its own sub-space via
 * `OdeHandle::createNewSimpleSpace(parent, true)` (true == ignore inside collisions).
 */
#ifndef MOR_RAYSENSOR_H
#define MOR_RAYSENSOR_H

#include "primitive.h"
#include "odehandle.h"
#include "globaldata.h"
#include "pose_types.h"
#include "sensor.h"

namespace mor {

class RaySensor : public Sensor {
public:
  /// @param range maximum sensing distance (ray length)
  explicit RaySensor(double range);
  ~RaySensor() override;

  /// attach the ray to `own` at local `pose` (the ray senses along pose's +Z)
  void init(const OdeHandle& odeHandle, Primitive* own, const Pose& pose);

  /* --- Sensor interface --- */
  int  getSensorNumber() const override { return 1; }
  /// latch the nearest hit for this step; call once per step before get()
  void sense(const GlobalData& globaldata) override;
  int  get(double* sensors, int maxlen) const override { if (maxlen<1) return 0; sensors[0]=len; return 1; }

  /// measured distance to the nearest object (== range if nothing in range)
  double get() const { return len; }
  /// the ray's collision geom — pass to OdeHandle::addIgnoredPair to exclude siblings
  dGeomID getGeom() const { return transform ? transform->getGeom() : nullptr; }

  /// (internal) called by the collision callback with each hit's depth
  void registerHit(double depth){ detection = std::min(detection, depth); }

private:
  double range;
  double len;        ///< latched measurement
  double detection;  ///< running min within the current step
  long   lasttime;
  Ray*       ray;
  Transform* transform;
  bool       initialised;
};

} // namespace mor
#endif
