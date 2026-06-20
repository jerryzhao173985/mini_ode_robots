/* mini_ode_robots — nimm4.cpp */
#include "nimm4.h"
#include "mor/primitive.h"
#include "mor/joint.h"

namespace mor {

Nimm4::Nimm4(const OdeHandle& odeHandle, const std::string& name)
  : OdeRobot(odeHandle, name) {}

void Nimm4::placeIntern(const Pose& pose){
  // chassis at the robot origin
  Box* chassis = new Box(length, width, cheight);
  chassis->init(odeHandle, chassisMass);
  chassis->setPose(pose);
  // A robot is an ACTIVE agent: never let its bodies auto-disable (a sleeping
  // body ignores joint-motor commands until a collision wakes it). The world
  // default auto-disable still applies to passive objects (debris, stacks).
  dBodySetAutoDisableFlag(chassis->getBody(), 0);
  objects.push_back(chassis);

  // four wheels: local positions (x fore/aft, y left/right), axle along local Y
  const double xs[4] = { +0.18, +0.18, -0.18, -0.18 };  // FL, FR, BL, BR
  const double ys[4] = { +0.20, -0.20, +0.20, -0.20 };
  // a cylinder's axis is local Z; rotate Z -> Y so the wheel rolls about Y (forward = X)
  const Pose zToY = Pose::rotate(Vec3(0,0,1), Vec3(0,1,0));

  for (int i=0;i<4;i++){
    Cylinder* wheel = new Cylinder(wheelRadius, wheelWidth);
    wheel->setSubstance(Substance::getRubber(20));      // grippy tyres
    wheel->init(odeHandle, wheelMass);
    dBodySetAutoDisableFlag(wheel->getBody(), 0);
    Pose localWheel = zToY * Pose::translate(xs[i], ys[i], 0.0);
    wheel->setPose(localWheel * pose);
    objects.push_back(wheel);

    // hinge: anchor at the wheel centre, axis = local Y carried into world by `pose`
    Pos  anchor   = Vec3(xs[i], ys[i], 0.0) * pose;
    Axis axleWorld(Axis(0,1,0) * pose);                  // direction (w=0): rotation only
    HingeJoint* hj = new HingeJoint(chassis, wheel, anchor, axleWorld);
    hj->init(odeHandle);                                 // ignores chassis<->wheel collision
    hj->setParam(dParamFMax, maxForce);                  // arm the motor (servo) with a force budget
    hj->setParam(dParamVel, 0.0);
    joints.push_back(hj);
  }
}

void Nimm4::setMotorsIntern(const double* motors, int n){
  if (n < 2) return;
  double leftCmd  = motors[0];
  double rightCmd = motors[1];
  // joints 0,2 -> left ; 1,3 -> right.  Servo via target angular velocity.
  static_cast<OneAxisJoint*>(joints[0])->setParam(dParamVel, leftCmd  * maxSpeed);
  static_cast<OneAxisJoint*>(joints[2])->setParam(dParamVel, leftCmd  * maxSpeed);
  static_cast<OneAxisJoint*>(joints[1])->setParam(dParamVel, rightCmd * maxSpeed);
  static_cast<OneAxisJoint*>(joints[3])->setParam(dParamVel, rightCmd * maxSpeed);
}

int Nimm4::getSensorsIntern(double* sensors, int n) const {
  if (n < 2) return 0;
  auto rate = [&](int j){ return static_cast<OneAxisJoint*>(joints[j])->getPosition1Rate(); };
  sensors[0] = 0.5 * (rate(0) + rate(2));   // average left wheel speed
  sensors[1] = 0.5 * (rate(1) + rate(3));   // average right wheel speed
  return 2;
}

} // namespace mor
