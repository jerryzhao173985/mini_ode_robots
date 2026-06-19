/* demo_snake.cpp — an undulating snake whose ANISOTROPIC friction turns a lateral
 * sine wave into net forward locomotion. Reports and checks the centroid travel. */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "mor/logger.h"
#include "snake.h"
#include <cstdio>
#include <cmath>
#include <vector>

using namespace mor;
static int failures = 0;
static void check(bool ok, const char* what){
  std::printf("  [%s] %s\n", ok?"PASS":"FAIL", what); if(!ok) failures++;
}
static Vec3 centroid(OdeRobot& r){
  Vec3 c; auto& ps = r.getAllPrimitives();
  for (auto* p : ps) c += p->getPosition();
  return c * (1.0 / ps.size());
}

int main(){
  Simulation sim; sim.init();
  Plane ground; ground.init(sim.getOdeHandle(), 0);

  const int N = 7;
  Snake snake(sim.getOdeHandle(), N, "Snake");
  snake.place(Pos(0,0, snake.bodyRadius()+0.005));
  sim.addRobot(&snake);

  sim.run(50);                       // settle onto the ground
  Vec3 start = centroid(snake);

  // record the centroid trajectory to a CSV for offline analysis (headless workflow)
  Logger log("snake_trajectory.csv", {"cx","cy"});

  // feed a traveling sine wave to the N-1 joints
  const int J = N - 1;
  const double amp = 0.7, freq = 0.9, phase = 1.1;
  bool nan = false;
  sim.run(2000, [&](Simulation& s, double t){
    double cmd[16];
    for (int i=0;i<J;i++) cmd[i] = amp * std::sin(2*M_PI*freq*t - i*phase);
    snake.setMotors(cmd, J);
    Vec3 c = centroid(snake);
    if (!std::isfinite(c.x())) nan = true;
    log.log(t, { c.x(), c.y() });
  });

  Vec3 end = centroid(snake);
  double travel = (end - start).length();
  std::printf("Snake: start=(%.2f,%.2f)  end=(%.2f,%.2f)  travel=%.3f m\n",
              start.x(),start.y(), end.x(),end.y(), travel);

  check(!nan && std::isfinite(travel), "snake stays finite (no explosion)");
  check(travel > 0.1, "undulation + anisotropic friction produce net locomotion");
  check(log.ok() && log.count() > 0, "trajectory logged to snake_trajectory.csv");
  std::printf("  logged %ld rows to snake_trajectory.csv\n", log.count());

  std::printf("\nSNAKE DEMO: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
