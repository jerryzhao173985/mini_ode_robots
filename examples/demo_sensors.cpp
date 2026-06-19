/* demo_sensors.cpp — composable sensing via OdeRobot::addSensor (Patch 1).
 *
 * Takes the stock Nimm4 and ATTACHES a forward RaySensor + a wheel JointSensor
 * without touching the robot's class. The robot's getSensors() then transparently
 * aggregates: [leftRate, rightRate, rayDistance, wheelAngle]. Verifies the channel
 * count, that the ray reads the true distance to a wall through the robot's API,
 * and that the reading shrinks as the robot drives toward the wall.
 */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "mor/joint.h"
#include "mor/raysensor.h"
#include "mor/jointsensor.h"
#include "nimm4.h"
#include <cstdio>
#include <cmath>

using namespace mor;
static int failures = 0;
static void check(bool ok, const char* what){
  std::printf("  [%s] %s\n", ok?"PASS":"FAIL", what); if(!ok) failures++;
}

int main(){
  Simulation sim; sim.init();
  OdeHandle& oh = sim.getOdeHandle();
  Plane ground; ground.init(oh, 0);

  // a wall ahead in the robot's forward (-X) drive direction; face at x = -1.9
  Box wall(0.2, 3.0, 0.6); wall.init(oh, 0, Primitive::Geom); wall.setPosition(Pos(-2.0, 0, 0.3));

  Nimm4 robot(oh, "Nimm4+sensors");
  robot.place(Pos(0,0, robot.restHeight()+0.005));
  sim.addRobot(&robot);

  // --- attach generic sensors AFTER placement (robot already built its parts) ---
  RaySensor* ray = new RaySensor(5.0);
  ray->init(oh, robot.getMainPrimitive(), Pose::rotate(Vec3(0,0,1), Vec3(-1,0,0))); // +Z -> -X (forward)
  robot.addSensor(ray);
  robot.addSensor(new JointSensor(static_cast<OneAxisJoint*>(robot.getAllJoints()[0]))); // a wheel angle

  check(robot.getSensorNumber() == 4, "getSensorNumber aggregates 2 intern + ray + joint = 4");
  check(robot.getMotorNumber()  == 2, "getMotorNumber unchanged (2 drive channels)");

  // settle, then read all channels through the robot's aggregating API
  double m0[2] = {0,0}; robot.setMotors(m0, 2);
  sim.run(60);
  robot.sense(sim.getGlobalData());
  double s[8] = {0}; int ns = robot.getSensors(s, 8);
  std::printf("  channels=%d  [Lrate=%.2f Rrate=%.2f ray=%.3f wheelAngle=%.3f]\n",
              ns, s[0], s[1], s[2], s[3]);
  check(ns == 4, "getSensors writes all 4 aggregated channels");
  check(std::fabs(s[2] - 1.9) < 0.35, "attached ray reads true distance to the wall (~1.9 m) via robot API");

  // drive forward; the forward ray distance must shrink as the robot approaches the wall
  double rayStart = s[2];
  double mf[2] = {1.0, 1.0}; robot.setMotors(mf, 2);
  sim.run(150);
  robot.sense(sim.getGlobalData());
  robot.getSensors(s, 8);
  std::printf("  after driving: ray=%.3f (was %.3f), robot x=%.3f\n", s[2], rayStart, robot.getPosition().x());
  check(s[2] < rayStart - 0.3, "ray distance shrinks as the robot drives toward the wall");
  check(std::isfinite(s[2]), "no NaN");

  std::printf("\nSENSOR DEMO: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
