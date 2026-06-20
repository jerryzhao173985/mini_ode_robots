/* demo_hexapod.cpp — a tripod-gait walk on the 6-legged Hexapod.
 *
 * The robot exposes 12 hip channels ([lift,swing] x 6 legs) + 6 foot contacts; the
 * GAIT lives here: two tripods 180 deg out of phase, each leg tracing a lift/swing
 * ellipse, so three feet always support the body. Verifies net forward locomotion
 * and logs the trunk trajectory.
 */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "mor/logger.h"
#include "hexapod.h"
#include <cstdio>
#include <cmath>

using namespace mor;
static int failures = 0;
static void check(bool ok, const char* what){
  std::printf("  [%s] %s\n", ok?"PASS":"FAIL", what); if(!ok) failures++;
}

int main(){
  Simulation sim; sim.init();
  Plane ground; ground.init(sim.getOdeHandle(), 0);
  Hexapod hex(sim.getOdeHandle());
  hex.place(Pos(0,0, hex.restHeight()));
  sim.addRobot(&hex);

  // let it settle into a stance
  double cmd[12] = {0}; hex.setMotors(cmd, 12);
  sim.run(80);
  Pos start = hex.getPosition();

  // tripod gait parameters
  const double freq = 1.3, liftAmp = 0.7, swingAmp = 0.8;
  Logger log("hexapod_trajectory.csv", {"x","y","z"});
  bool nan = false;

  sim.run(3000, [&](Simulation& s, double t){
    for (int n = 0; n < Hexapod::NLEG; n++){
      double phi = 2*M_PI*freq*t + (Hexapod::tripodOf(n) ? M_PI : 0.0);
      // lift up (foot off ground) on the first half of the cycle; swing leads it
      cmd[2*n]   = liftAmp  * std::cos(phi);     // lift  (+ = up / off ground)
      cmd[2*n+1] = swingAmp * std::sin(phi);     // swing (fore/aft)
    }
    hex.setMotors(cmd, 12);
    Pos p = hex.getPosition();
    if (!std::isfinite(p.x())) nan = true;
    log.log(t, { p.x(), p.y(), p.z() });
  });

  Pos end = hex.getPosition();
  double travel = std::sqrt((end.x()-start.x())*(end.x()-start.x()) + (end.y()-start.y())*(end.y()-start.y()));
  std::printf("Hexapod: start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f) travel=%.3f m, upright z=%.2f\n",
              start.x(),start.y(),start.z(), end.x(),end.y(),end.z(), travel, end.z());

  check(!nan, "stays finite (no explosion)");
  check(end.z() > 0.18, "stays standing (did not collapse)");
  check(travel > 0.3, "tripod gait produces net locomotion");
  check(log.ok() && log.count() > 0, "trajectory logged to hexapod_trajectory.csv");

  std::printf("\nHEXAPOD DEMO: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
