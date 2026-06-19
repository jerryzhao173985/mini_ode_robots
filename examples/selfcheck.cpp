/* selfcheck.cpp — headless physics verification.
 *
 * "It compiled and ran" is NOT verification (ODE skill: verifying-simulations.md).
 * We assert physical invariants that a CORRECT engine must satisfy and that a
 * broken one violates:
 *   1. Drop & settle : a dropped body never gains energy, and comes to rest.
 *   2. Energy        : translational+potential energy never exceeds the start.
 *   3. Hinge constraint: a pendulum bob stays at a fixed distance from its pivot.
 *   4. Stack         : stacked boxes don't explode and come to rest.
 * Any NaN anywhere is a hard failure.
 */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "mor/joint.h"
#include <cstdio>
#include <cmath>

using namespace mor;
static int failures = 0;
static void check(bool ok, const char* what){
  std::printf("  [%s] %s\n", ok?"PASS":"FAIL", what);
  if (!ok) failures++;
}
static bool finite3(const Vec3& v){ return std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z()); }

/* ---------------- 1 & 2: drop a sphere, watch energy ---------------- */
static void test_drop_and_energy(){
  std::printf("[1+2] drop & energy\n");
  Simulation sim; sim.init();
  OdeHandle& oh = sim.getOdeHandle();
  const double g = std::fabs(sim.getGlobalData().odeConfig.gravity);

  Plane ground; ground.init(oh, 0);                     // static ground at z=0

  const double r = 0.3, m = 1.0, z0 = 2.0;
  Sphere ball(r); ball.init(oh, m);
  ball.setPosition(Pos(0,0,z0));
  ball.setSubstance(Substance::getRubber(15));          // a little bouncy

  double E0 = m*g*z0;                                    // starts at rest: all potential
  double maxZ = z0, maxE = E0;
  bool nan=false;
  for (int i=0;i<800;i++){                               // 8 s at dt=0.01
    sim.step();
    Pos p = ball.getPosition(), v = ball.getVel();
    if (!finite3(p) || !finite3(v)) { nan=true; break; }
    double E = 0.5*m*(v*v) + m*g*p.z();                  // translational + potential (<= total)
    if (p.z() > maxZ) maxZ = p.z();
    if (E > maxE) maxE = E;
  }
  Pos pf = ball.getPosition(), vf = ball.getVel();
  check(!nan, "no NaN while falling/bouncing");
  check(maxZ <= z0 + 1e-3, "never rises above drop height (no energy creation)");
  check(maxE <= E0 + 1e-6*E0 + 1e-9, "trans+potential energy never exceeds start");
  check(std::fabs(pf.z() - r) < 0.02, "comes to rest on the ground (z ~= radius)");
  check(vf.length() < 0.05, "settles to rest (|v| ~ 0)");
  // RAII teardown: ~Simulation closes ODE *after* the bodies declared above are
  // destroyed (sim is declared first => destroyed last). Calling close() here,
  // while those bodies are still alive, would dGeomDestroy freed handles -> crash.
}

/* ---------------- 3: hinge pendulum keeps its constraint ---------------- */
static void test_hinge_constraint(){
  std::printf("[3] hinge pendulum constraint\n");
  Simulation sim; sim.init();
  sim.setGravity(-9.81);
  OdeHandle& oh = sim.getOdeHandle();

  // a static anchor body at the pivot
  Pos pivot(0,0,2.0);
  Box anchor(0.1,0.1,0.1); anchor.init(oh, 1.0, Primitive::Body); // body only, no geom
  anchor.setPosition(pivot);
  dBodySetKinematic(anchor.getBody());                  // immovable pivot

  // the bob, offset horizontally so it will swing
  Pos bobPos = pivot + Vec3(0.8,0,0);
  Sphere bob(0.1); bob.init(oh, 1.0);
  bob.setPosition(bobPos);

  HingeJoint hinge(&anchor, &bob, pivot, Axis(0,1,0));
  hinge.init(oh);

  double L0 = (bob.getPosition() - anchor.getPosition()).length();
  double maxErr = 0; bool nan=false; double maxReach=0;
  for (int i=0;i<600;i++){
    sim.step();
    Pos pb = bob.getPosition();
    if (!finite3(pb)) { nan=true; break; }
    double L = (pb - anchor.getPosition()).length();
    maxErr = std::max(maxErr, std::fabs(L-L0));
    maxReach = std::max(maxReach, pb.x());              // it must actually swing
  }
  check(!nan, "no NaN in pendulum");
  check(maxErr < 5e-3, "bob stays at fixed distance from pivot (constraint satisfied)");
  check(maxReach > 0.7, "pendulum actually swings (not frozen)");
  // RAII teardown: ~Simulation closes ODE *after* the bodies declared above are
  // destroyed (sim is declared first => destroyed last). Calling close() here,
  // while those bodies are still alive, would dGeomDestroy freed handles -> crash.
}

/* ---------------- 4: a small box stack stays put ---------------- */
static void test_stack(){
  std::printf("[4] box stack stability\n");
  Simulation sim; sim.init();
  OdeHandle& oh = sim.getOdeHandle();
  Plane ground; ground.init(oh, 0);

  const double s = 0.4;            // box side
  Box* boxes[3];
  for (int i=0;i<3;i++){
    boxes[i] = new Box(s,s,s);
    boxes[i]->init(oh, 1.0);
    boxes[i]->setPosition(Pos(0,0, s/2 + i*s));   // stacked, touching
  }
  bool nan=false;
  for (int i=0;i<600;i++){
    sim.step();
    for (auto* b : boxes) if (!finite3(b->getPosition())) nan=true;
    if (nan) break;
  }
  bool atRest=true, nearStack=true;
  for (int i=0;i<3;i++){
    Pos p = boxes[i]->getPosition(), v = boxes[i]->getVel();
    if (v.length() > 0.1) atRest=false;
    if (std::fabs(p.x())>0.2 || std::fabs(p.y())>0.2) nearStack=false;  // didn't topple/slide
    if (std::fabs(p.z() - (s/2 + i*s)) > 0.1) nearStack=false;          // kept its layer
  }
  check(!nan, "no NaN in stack");
  check(nearStack, "boxes keep their stacked positions (no explosion/topple)");
  check(atRest, "stack comes to rest");
  for (auto* b : boxes) delete b;
  // RAII teardown: ~Simulation closes ODE *after* the bodies declared above are
  // destroyed (sim is declared first => destroyed last). Calling close() here,
  // while those bodies are still alive, would dGeomDestroy freed handles -> crash.
}

int main(){
  test_drop_and_energy();
  test_hinge_constraint();
  test_stack();
  std::printf("\nSELF-CHECK: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
