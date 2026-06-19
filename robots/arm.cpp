/* mini_ode_robots — arm.cpp */
#include "arm.h"
#include "mor/primitive.h"
#include "mor/joint.h"
#include <cmath>

namespace mor {

Arm::Arm(const OdeHandle& odeHandle, int numSegments, const std::string& name)
  : OdeRobot(odeHandle, name), nSeg(numSegments) {
  targets.assign(nSeg, 0.0);
}

void Arm::placeIntern(const Pose& pose){
  // capsule's long axis is local Z; align it with local X (arm grows along +X)
  const Pose zToX = Pose::rotate(Vec3(0,0,1), Vec3(1,0,0));

  // static base (geom only) at the robot origin — anchors the arm to the world
  Box* base = new Box(0.12,0.12,0.12);
  base->init(odeHandle, 0, Primitive::Geom);
  base->setPose(pose);
  objects.push_back(base);

  Primitive* prev = base;       // hinge part2 (the base has no body -> hinged to world)
  servos.clear();
  for (int i=0;i<nSeg;i++){
    Capsule* seg = new Capsule(segRad, segLen);
    seg->init(odeHandle, segMass);
    dBodySetAutoDisableFlag(seg->getBody(), 0);          // active agent
    // segment i spans local x in [i*L, (i+1)*L]; centre at (i+0.5)*L
    Pose local = zToX * Pose::translate((i + 0.5) * segLen, 0, 0);
    seg->setPose(local * pose);
    objects.push_back(seg);

    // hinge at the segment's near end, axis = local Y (bends in the X-Z plane)
    Pos  anchor = Vec3(i * segLen, 0, 0) * pose;
    Axis axisY(Axis(0,1,0) * pose);
    HingeJoint* hj = new HingeJoint(seg, prev, anchor, axisY);
    hj->init(odeHandle);
    joints.push_back(hj);

    // a velocity-based position servo on this joint (travel +/- ~80 deg)
    servos.emplace_back(hj, -1.4, 1.4, power, /*damp*/0.3, /*maxVel*/12.0);
    prev = seg;
  }
}

void Arm::setMotorsIntern(const double* motors, int n){
  for (int i=0;i<nSeg && i<n;i++) targets[i] = motors[i];
}

void Arm::doInternalStuffIntern(GlobalData&){
  // servos must be driven every physics step (they emit a PID torque each call)
  for (int i=0;i<nSeg;i++) servos[i].set(targets[i]);
}

int Arm::getSensorsIntern(double* sensors, int n){
  int m = 0;
  for (int i=0;i<nSeg && i<n;i++) sensors[m++] = servos[i].get();
  return m;
}

} // namespace mor
