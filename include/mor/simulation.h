/* mini_ode_robots — simulation.h
 *
 * Owns the ODE lifecycle (dInitODE2 .. dCloseODE) and runs the canonical loop:
 *   per step:  dSpaceCollide -> nearCallback -> dWorldStep -> dJointGroupEmpty
 * The nearCallback recovers each geom's Substance (via dGeomGetData) and builds
 * the contact surface from the two materials. Extracted from lpzrobots
 * simulation.cpp (odeStep + nearCallback), minus GUI/threading/rendering.
 */
#ifndef MOR_SIMULATION_H
#define MOR_SIMULATION_H

#include <ode/ode.h>
#include <vector>
#include <functional>
#include "odehandle.h"
#include "globaldata.h"
#include "pose_types.h"

namespace mor {

class OdeRobot;

class Simulation {
public:
  Simulation();
  ~Simulation();

  /// init ODE, create world/space/contactgroup, apply OdeConfig defaults.
  void init();
  /// teardown (also called by destructor if still open).
  void close();

  OdeHandle&  getOdeHandle()  { return odeHandle; }
  GlobalData& getGlobalData() { return globalData; }
  double      getTime() const { return globalData.time; }
  long        getStepCount() const { return stepCount; }
  /// number of contact points generated in the most recent step (for ground-contact sensing)
  long        getLastStepNumContacts() const { return lastNumContacts; }

  void setGravity(double g);                 ///< gravity along -Z (convenience)
  void setGravity(const Vec3& g);            ///< full gravity vector (e.g. tilt the world)
  void addRobot(OdeRobot* r) { robots.push_back(r); }

  /// One physics step. Returns false if the stepper reported an allocation failure.
  bool step();

  /** Run `steps` physics steps. `control(sim, time)` is invoked every
      controlInterval steps (after sensing, before actuation). */
  void run(long steps, const std::function<void(Simulation&, double)>& control = {});

  /* the C-function-pointer collision callbacks (data == this) */
  static void nearCallback(void* data, dGeomID o1, dGeomID o2);

private:
  OdeHandle              odeHandle;
  GlobalData             globalData;
  std::vector<OdeRobot*> robots;
  long                   stepCount;
  long                   lastNumContacts = 0;
  bool                   initialized;
};

} // namespace mor
#endif
