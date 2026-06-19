/* mini_ode_robots — snake.cpp */
#include "snake.h"
#include "mor/primitive.h"
#include "mor/joint.h"
#include <cmath>

namespace mor {

Snake::Snake(const OdeHandle& odeHandle, int numSegments, const std::string& name)
  : OdeRobot(odeHandle, name), nSeg(numSegments) {
  targets.assign(nSeg - 1, 0.0);
}

void Snake::placeIntern(const Pose& pose){
  const Pose zToX = Pose::rotate(Vec3(0,0,1), Vec3(1,0,0));  // capsule long axis: Z -> X

  Primitive* prev = nullptr;
  servos.clear();
  for (int i=0;i<nSeg;i++){
    Capsule* seg = new Capsule(segRad, segLen);
    // anisotropic friction: low along the body's long axis (local Z after zToX),
    // full laterally — this is what turns undulation into forward thrust.
    Substance s = Substance::getRubber(20);                 // grippy base
    s.toAnisotropFriction(frictionRatio, Axis(0,0,1));      // axis = capsule long axis (local Z)
    seg->setSubstance(s);
    seg->init(odeHandle, segMass);
    dBodySetAutoDisableFlag(seg->getBody(), 0);

    Pose local = zToX * Pose::translate(i * segLen, 0, 0);
    seg->setPose(local * pose);
    objects.push_back(seg);

    if (prev){
      // hinge between segment i-1 and i, axis = vertical (yaw) so it bends in the ground plane
      Pos  anchor = Vec3((i - 0.5) * segLen, 0, 0) * pose;
      Axis axisZ(Axis(0,0,1) * pose);
      HingeJoint* hj = new HingeJoint(seg, prev, anchor, axisZ);
      hj->init(odeHandle);
      joints.push_back(hj);
      // Intentionally a COMPLIANT force servo (OneAxisServo), not the stiff velocity servo:
      // a snake wants springy joints that yield to ground reaction. Audited as bounded
      // (tail oscillation ~0.06 rad at this gain, no blow-up over 30 s). maxVel>0 enables the
      // body velocity clamp (matches lpzrobots SchlangeServo).
      servos.emplace_back(hj, -1.0, 1.0, power, /*damp*/0.4, /*KI*/0.0, /*maxVel*/8.0);
    }
    prev = seg;
  }
}

void Snake::setMotorsIntern(const double* motors, int n){
  for (int i=0;i<(int)targets.size() && i<n;i++) targets[i] = motors[i];
}

void Snake::doInternalStuffIntern(GlobalData&){
  for (size_t i=0;i<servos.size();i++) servos[i].set(targets[i]);
}

int Snake::getSensorsIntern(double* sensors, int n){
  int m = 0;
  for (size_t i=0;i<servos.size() && m<n;i++) sensors[m++] = servos[i].get();
  return m;
}

} // namespace mor
