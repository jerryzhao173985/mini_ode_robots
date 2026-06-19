/* demo_arm.cpp — drive the servo arm and verify position control:
 * commanding +target then -target must move the tip and flip the joint-angle signs. */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "arm.h"
#include <cstdio>
#include <cmath>

using namespace mor;
static int failures = 0;
static void check(bool ok, const char* what){
  std::printf("  [%s] %s\n", ok?"PASS":"FAIL", what); if(!ok) failures++;
}

int main(){
  Simulation sim; sim.init();
  const int N = 3;
  Arm arm(sim.getOdeHandle(), N, "Arm");
  arm.place(Pos(0,0,1.0));        // static base at 1 m; links extend along +X
  sim.addRobot(&arm);

  auto driveTo = [&](double t){
    double cmd[8]; for(int i=0;i<N;i++) cmd[i]=t;
    arm.setMotors(cmd, N);
    sim.run(500);
  };

  driveTo(+0.5);
  Pos tipUp = arm.getAllPrimitives().back()->getPosition();
  double angUp[8]; arm.getSensors(angUp, N);

  driveTo(-0.5);
  Pos tipDown = arm.getAllPrimitives().back()->getPosition();
  double angDown[8]; arm.getSensors(angDown, N);

  double tipMove = (tipUp - tipDown).length();
  std::printf("Arm: tip(+cmd)=(%.2f,%.2f,%.2f)  tip(-cmd)=(%.2f,%.2f,%.2f)  |move|=%.3f\n",
              tipUp.x(),tipUp.y(),tipUp.z(), tipDown.x(),tipDown.y(),tipDown.z(), tipMove);
  std::printf("Arm: joint angles  +cmd[0]=%.2f  -cmd[0]=%.2f\n", angUp[0], angDown[0]);

  check(tipMove > 0.2, "tip moves substantially between +cmd and -cmd (servo position control)");
  check(angUp[0] > angDown[0] + 0.2, "joint angle follows command sign");
  check(std::isfinite(tipMove), "no NaN");

  std::printf("\nARM DEMO: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
