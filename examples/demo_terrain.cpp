/* demo_terrain.cpp — the Hexapod walking on rough HEIGHTFIELD ground.
 *
 * Builds a procedural bumpy terrain (an ODE heightfield wrapped as a static Primitive),
 * stands the hexapod on it at the correct local height, and runs the tripod gait. Verifies
 * the robot rests ON the terrain (doesn't fall through), then traverses it upright.
 */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "mor/heightfield.h"
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

  // gentle, long-wavelength rolling terrain — walkable by an open-loop tripod gait
  auto hfn = [](double x, double y){ return 0.015*std::sin(0.5*x)*std::cos(0.5*y); };
  HeightField terrain(14, 14, 112, 112, hfn, -0.05, 0.05);
  terrain.init(sim.getOdeHandle(), 0);

  Hexapod hex(sim.getOdeHandle());
  double z0 = terrain.heightAt(0,0) + hex.restHeight() + 0.02;   // stand ON the terrain
  hex.place(Pos(0,0,z0));
  sim.addRobot(&hex);

  double cmd[12]={0}; hex.setMotors(cmd,12);
  sim.run(120);                                    // settle onto the terrain
  Pos rest = hex.getPosition();
  double clearance = rest.z() - terrain.heightAt(rest.x(), rest.y());
  std::printf("settled: pos=(%.2f,%.2f,%.2f) terrainH=%.3f clearance=%.3f\n",
              rest.x(),rest.y(),rest.z(), terrain.heightAt(rest.x(),rest.y()), clearance);
  check(clearance > 0.1 && std::isfinite(rest.z()), "hexapod rests ON the terrain (did not fall through)");

  // tripod gait across the rough ground
  Pos start = hex.getPosition();
  const double freq=1.3, liftAmp=0.7, swingAmp=0.8;     // the gait proven stable on flat ground
  bool nan=false;
  sim.run(2500, [&](Simulation& s, double t){
    for (int n=0;n<Hexapod::NLEG;n++){
      double phi = 2*M_PI*freq*t + (Hexapod::tripodOf(n)?M_PI:0.0);
      cmd[2*n]   = liftAmp*std::cos(phi);
      cmd[2*n+1] = swingAmp*std::sin(phi);
    }
    hex.setMotors(cmd,12);
    if(!std::isfinite(hex.getPosition().x())) nan=true;
  });
  Pos end = hex.getPosition();
  double travel = std::sqrt((end.x()-start.x())*(end.x()-start.x())+(end.y()-start.y())*(end.y()-start.y()));
  std::printf("after walking on terrain: travel=%.3f m, upright z=%.2f (terrainH=%.3f)\n",
              travel, end.z(), terrain.heightAt(end.x(),end.y()));

  check(!nan, "stays finite on rough ground");
  check(end.z() - terrain.heightAt(end.x(),end.y()) > 0.1, "stays standing on the terrain (not collapsed)");
  check(travel > 0.2, "traverses the rough terrain (net locomotion)");

  std::printf("\nTERRAIN DEMO: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
