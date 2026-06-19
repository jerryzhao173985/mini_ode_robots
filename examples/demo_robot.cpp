/* demo_robot.cpp — drive the Nimm4 differential-drive robot, headless, and
 * verify it behaves like a robot: equal wheel speeds drive it forward; a
 * left/right speed difference turns it. (ODE skill: trajectory sanity check.)
 */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "nimm4.h"
#include <cstdio>
#include <cmath>

using namespace mor;
static int failures = 0;
static void check(bool ok, const char* what){
  std::printf("  [%s] %s\n", ok?"PASS":"FAIL", what); if(!ok) failures++;
}
// heading (yaw) of the chassis: angle of its local +X axis in the world XY-plane
static double heading(OdeRobot& r){
  Pose p = r.getMainPrimitive()->getPose();
  Vec3 fwd = Vec3(1,0,0) * p;            // includes translation...
  Vec3 org = Vec3(0,0,0) * p;
  Vec3 d = fwd - org;                    // ...so subtract origin to get the direction
  return std::atan2(d.y(), d.x());
}

int main(){
  Simulation sim; sim.init();
  OdeHandle& oh = sim.getOdeHandle();

  Plane ground; ground.init(oh, 0);

  Nimm4 robot(oh, "Nimm4");
  robot.place(Pos(0,0, robot.restHeight()+0.005));
  sim.addRobot(&robot);

  // let it settle onto its wheels
  double m_settle[2] = {0,0};
  robot.setMotors(m_settle, 2);
  sim.run(50);

  Pos  startPos = robot.getPosition();
  double startYaw = heading(robot);

  // --- phase 1: drive straight (both wheels forward) ---
  double m_fwd[2] = {1.0, 1.0};
  robot.setMotors(m_fwd, 2);
  sim.run(400);                               // 4 s
  Pos  midPos = robot.getPosition();
  double midYaw = heading(robot);
  double fwdDist = (midPos - startPos).length();
  double yawDriftStraight = std::fabs(midYaw - startYaw);

  // --- phase 2: turn in place-ish (wheels opposed) ---
  double m_turn[2] = {1.0, -1.0};
  robot.setMotors(m_turn, 2);
  sim.run(400);                               // 4 s
  double endYaw = heading(robot);
  double turned = std::fabs(endYaw - midYaw);
  if (turned > M_PI) turned = 2*M_PI - turned; // wrap

  std::printf("Nimm4 trajectory:\n");
  std::printf("  forward distance     = %.3f m\n", fwdDist);
  std::printf("  yaw drift (straight) = %.3f rad\n", yawDriftStraight);
  std::printf("  yaw change (turning) = %.3f rad\n", turned);
  std::printf("  final position       = (%.3f, %.3f, %.3f)\n",
              robot.getPosition().x(), robot.getPosition().y(), robot.getPosition().z());

  check(fwdDist > 0.8, "drives forward when both wheels spin forward");
  check(yawDriftStraight < 0.25, "drives roughly straight (small heading drift)");
  check(turned > 0.5, "turns when wheels are driven in opposite directions");
  check(std::isfinite(fwdDist) && robot.getPosition().z() > 0.0, "robot stays upright / no NaN");

  // sim closes via RAII (declared before `robot`/`ground` => destroyed after them)
  std::printf("\nROBOT DEMO: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
