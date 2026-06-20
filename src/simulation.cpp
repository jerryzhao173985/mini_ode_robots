/* mini_ode_robots — simulation.cpp
 * The ODE lifecycle + canonical step loop + substance-driven collision callback.
 */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "mor/substance.h"
#include "mor/oderobot.h"
#include <cstdio>
#include <cassert>

namespace mor {

Simulation::Simulation() : stepCount(0), initialized(false) {}
Simulation::~Simulation(){ if (initialized) close(); }

void Simulation::init(){
  dInitODE2(0);                                       // skill rule #2: init before anything
  assert(dCheckConfiguration("ODE_double_precision")  // skill rule #7
         && "this engine expects a double-precision libode");
  dWorldID world = dWorldCreate();
  dSpaceID space = dHashSpaceCreate(0);
  dSpaceSetCleanup(space, 0);
  dJointGroupID jg = dJointGroupCreate(0);
  odeHandle = OdeHandle(world, space, jg);
  odeHandle.timePtr = &globalData.time;   // servos/sensors read the clock via odeHandle.getTime()

  // world defaults — set BEFORE bodies are created (skill rule #9)
  dWorldSetGravity(world, globalData.odeConfig.gravityX, globalData.odeConfig.gravityY,
                          globalData.odeConfig.gravity);
  dWorldSetCFM(world, globalData.odeConfig.cfm);
  dWorldSetERP(world, globalData.odeConfig.erp);
  // contact-stability knobs (cf. lpzrobots simulation.cpp:225-226): cap correcting velocity
  // (safe default) and apply the opt-in surface layer (default 0 — see globaldata.h for why).
  dWorldSetContactMaxCorrectingVel(world, globalData.odeConfig.contactMaxCorrectingVel);
  dWorldSetContactSurfaceLayer(world, globalData.odeConfig.contactSurfaceLayer);
  dWorldSetAutoDisableFlag(world, globalData.odeConfig.autoDisable ? 1 : 0);  // resting bodies sleep
  dWorldSetQuickStepNumIterations(world, globalData.odeConfig.quickStepIters);
  globalData.time = 0.0;
  stepCount = 0;
  initialized = true;
  odeLifecycleAcquire();   // mark ODE alive so Primitive/Joint dtors free safely
}

void Simulation::close(){
  if (!initialized) return;
  dJointGroupDestroy(odeHandle.jointGroup);
  dSpaceDestroy(odeHandle.space);
  dWorldDestroy(odeHandle.world);
  dCloseODE();
  initialized = false;
  // After this, any surviving Primitive/Joint must NOT touch ODE (see odeIsAlive).
  odeLifecycleRelease();
}

void Simulation::setGravity(double g){
  globalData.odeConfig.gravityX = 0; globalData.odeConfig.gravityY = 0;
  globalData.odeConfig.gravity = g;
  if (initialized) dWorldSetGravity(odeHandle.world, 0, 0, g);
}
void Simulation::setGravity(const Vec3& g){
  globalData.odeConfig.gravityX = g.x(); globalData.odeConfig.gravityY = g.y();
  globalData.odeConfig.gravity  = g.z();   // OdeConfig now reflects the full vector
  if (initialized) dWorldSetGravity(odeHandle.world, g.x(), g.y(), g.z());
}

// The canonical near-callback, specialized with nested-space recursion and substances.
void Simulation::nearCallback(void* data, dGeomID o1, dGeomID o2){
  Simulation* me = static_cast<Simulation*>(data);
  if (!me) return;

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2)){           // a space vs something -> recurse
    dSpaceCollide2(o1, o2, data, &nearCallback);
    return;
  }
  if (me->odeHandle.isIgnoredPair(o1, o2)) return;     // joint-connected parts don't collide

  Primitive* p1 = (Primitive*)dGeomGetData(o1);
  Primitive* p2 = (Primitive*)dGeomGetData(o2);
  if (!p1 || !p2) return;                              // not one of ours

  const int N = 128;
  dContact contact[N];                                 // an ARRAY (skill rule #4)
  // Primitive pairs yield <=4 contacts; a HeightField/TriMesh pair can produce many, so N
  // is sized with headroom and saturation is reported (silent truncation would read as
  // "all contacts handled" — the canonical heightfield bug).
  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  if (n <= 0) return;
  if (n == N){
    static bool warned = false;
    if (!warned){ std::fprintf(stderr, "mor: contact array saturated at %d (raise N for dense terrain)\n", N); warned = true; }
  }

  const Substance& s1 = p1->substance;
  const Substance& s2 = p2->substance;
  dSurfaceParameters surf;
  int rv = 1;
  if (s1.callback) rv = s1.callback(surf, me->globalData, s1.userdata, contact, n, o1, o2, s1, s2);
  if (s2.callback && rv==1) rv = s2.callback(surf, me->globalData, s2.userdata, contact, n, o2, o1, s2, s1);
  if (rv==1) Substance::getSurfaceParams(surf, s1, s2, me->globalData.odeConfig.simStepSize);
  if (rv==0) return;                                   // a callback vetoed the contact

  for (int i=0;i<n;i++){
    contact[i].surface = surf;
    dJointID c = dJointCreateContact(me->odeHandle.world, me->odeHandle.jointGroup, &contact[i]);
    dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
  }
  me->lastNumContacts += n;   // tally for getLastStepNumContacts()
}

bool Simulation::step(){
  lastNumContacts = 0;                                  // reset per-step contact tally
  // a. broadphase + narrowphase -> contact joints
  dSpaceCollide(odeHandle.space, this, &nearCallback);
  for (dSpaceID s : odeHandle.getSpaces())             // internal collisions of registered sub-spaces
    dSpaceCollide(s, this, &nearCallback);

  // b. integrate one fixed timestep (skill rule #5: check the return)
  int ok;
  if (globalData.odeConfig.useQuickStep)
    ok = dWorldQuickStep(odeHandle.world, globalData.odeConfig.simStepSize);
  else
    ok = dWorldStep(odeHandle.world, globalData.odeConfig.simStepSize);

  // c. discard this step's contact joints (skill rule #3)
  dJointGroupEmpty(odeHandle.jointGroup);

  globalData.time += globalData.odeConfig.simStepSize;
  globalData.sim_step = ++stepCount;
  return ok != 0;
}

void Simulation::run(long steps, const std::function<void(Simulation&,double)>& control){
  for (long i=0;i<steps;i++){
    bool doControl = (i % globalData.odeConfig.controlInterval) == 0;
    if (doControl){
      for (OdeRobot* r : robots) r->sense(globalData);
      if (control) control(*this, globalData.time);
    }
    for (OdeRobot* r : robots) r->doInternalStuff(globalData);  // motors act here
    if (!step()){
      std::fprintf(stderr, "Simulation::run: stepper failed at step %ld\n", i);
      return;
    }
  }
}

} // namespace mor
