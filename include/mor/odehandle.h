/* mini_ode_robots — odehandle.h
 *
 * The context bundle: world + collision space + contact-joint group + default
 * material, plus the two collision-filtering mechanisms robots need:
 *   - ignored geom pairs   (joint-connected parts must not collide)
 *   - nested spaces        (a robot can own a sub-space)
 *
 * Mirrors lpzrobots' utils/odehandle.h, minus the selforg HashSet (we use
 * std::unordered_set) and minus the ODE lifecycle (that lives in Simulation).
 * Copyable by value; copies SHARE the same ignored-pair set / space list.
 */
#ifndef MOR_ODEHANDLE_H
#define MOR_ODEHANDLE_H

#include <ode/ode.h>
#include <vector>
#include <memory>
#include <unordered_set>
#include <utility>
#include <functional>
#include <cstdint>
#include "substance.h"

namespace mor {

class Primitive;

/* ODE lifecycle guard ------------------------------------------------------
 * Primitive/Joint destructors call dGeomDestroy/dBodyDestroy/dJointDestroy,
 * which are only valid while an ODE world is alive. Simulation acquires on
 * init() and releases on close(). Destructors consult odeIsAlive() so that
 * destroying a Primitive/Joint AFTER its Simulation (wrong RAII order) is SAFE
 * (it simply skips the ODE-free, leaking only at process exit) instead of UB. */
void odeLifecycleAcquire();
void odeLifecycleRelease();
bool odeIsAlive();

class OdeHandle {
public:
  dWorldID      world;
  dSpaceID      space;
  dJointGroupID jointGroup;
  Substance     substance;   ///< default material handed to primitives made with this handle
  double*       timePtr;     ///< points at Simulation's clock (for servos/sensors). May be null.

  /// current simulation time (0 if not wired to a Simulation)
  double getTime() const { return timePtr ? *timePtr : 0.0; }

  OdeHandle();
  OdeHandle(dWorldID world, dSpaceID space, dJointGroupID jointGroup);

  /* --- (sub)spaces for collision detection ---
     A robot can live in its own sub-space: cheaper broadphase, and self-collisions
     can be globally ignored. NOTE: dSpaceCollide is non-recursive, so EVERY sub-space
     whose internal collisions must be tested has to be registered (addSpace, done for
     you by createNew*Space when ignore_inside_collisions==false). Nesting does not
     auto-register descendants — keep the layout flat (one sub-space per articulated body). */
  void addSpace(dSpaceID s);
  void removeSpace(dSpaceID s);
  const std::vector<dSpaceID>& getSpaces() const { return *spaces; }

  /// give this handle a fresh simple sub-space under `parentspace`; if
  /// ignore_inside_collisions is false it is registered so its internal pairs collide.
  void createNewSimpleSpace(dSpaceID parentspace, bool ignore_inside_collisions);
  /// like createNewSimpleSpace but a hash space (better for many objects).
  void createNewHashSpace(dSpaceID parentspace, bool ignore_inside_collisions);
  /// destroy this handle's space and unregister it.
  void deleteSpace();

  /* --- ignored geom pairs (joint-connected parts must not collide) ---
     Keyed on the actual geom pointers (not a lossy hash). CONTRACT: if you call
     addIgnoredPair manually, removeIgnoredPair before destroying either geom, or a
     stale pair could match a future geom that reuses the freed address. The Joint
     path does this for you (Joint::~Joint removes its pair). */
  void addIgnoredPair(dGeomID g1, dGeomID g2);
  void addIgnoredPair(Primitive* p1, Primitive* p2);
  void removeIgnoredPair(dGeomID g1, dGeomID g2);
  void removeIgnoredPair(Primitive* p1, Primitive* p2);
  bool isIgnoredPair(dGeomID g1, dGeomID g2) const {
    return ignoredPairs->count(std::make_pair(g1,g2)) || ignoredPairs->count(std::make_pair(g2,g1));
  }

private:
  struct GeomPairHash {
    size_t operator()(const std::pair<dGeomID,dGeomID>& p) const {
      size_t a = std::hash<const void*>()(p.first), b = std::hash<const void*>()(p.second);
      return a * 0x9E3779B97F4A7C15ull ^ b;
    }
  };
  std::shared_ptr<std::unordered_set<std::pair<dGeomID,dGeomID>, GeomPairHash>> ignoredPairs;
  std::shared_ptr<std::vector<dSpaceID>>                                        spaces;
};

} // namespace mor
#endif
