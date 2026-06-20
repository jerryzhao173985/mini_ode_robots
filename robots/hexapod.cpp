/* mini_ode_robots — hexapod.cpp */
#include "hexapod.h"
#include "mor/primitive.h"
#include "mor/joint.h"
#include "mor/contactsensor.h"
#include <cmath>

namespace mor {

Hexapod::Hexapod(const OdeHandle& odeHandle, const std::string& name)
  : OdeRobot(odeHandle, name) {}

void Hexapod::placeIntern(const Pose& pose){
  // trunk at the robot origin (objects[0] -> getMainPrimitive tracks it)
  Box* trunk = new Box(bodyL, bodyW, bodyH);
  trunk->init(odeHandle, bodyMass);
  trunk->setPose(pose);
  dBodySetAutoDisableFlag(trunk->getBody(), 0);
  objects.push_back(trunk);

  // downward tilt that lands the foot ~on the ground from hip height standH
  double dz = -standH / legLen;                 // required vertical fraction
  if (dz < -0.95) dz = -0.95;
  double dy = std::sqrt(1.0 - dz*dz);           // outward fraction
  hips.clear();

  for (int n = 0; n < NLEG; n++){
    double side = (n % 2 == 0) ? +1.0 : -1.0;   // +Y left / -Y right
    int    col  = n / 2;                         // 0 front, 1 mid, 2 back
    double xcol = (1 - col) * (bodyL * 0.35);    // front +x ... back -x

    Vec3 hipLocal(xcol, side * bodyW/2, 0);
    Vec3 dLocal(0, side * dy, dz);               // leg direction: outward + down (unit)

    Capsule* leg = new Capsule(legR, legLen);
    leg->init(odeHandle, legMass);
    dBodySetAutoDisableFlag(leg->getBody(), 0);
    // capsule local +Z -> dLocal, centre at hip + halfLen*dLocal, all carried by `pose`
    Pose legLocal = Pose::rotate(Vec3(0,0,1), dLocal)
                  * Pose::translate(hipLocal + dLocal * (0.5*legLen));
    leg->setPose(legLocal * pose);
    objects.push_back(leg);

    // 2-DOF hip: axis1 = lift (body X), axis2 = swing (body Z) — perpendicular
    Pos  anchor   = hipLocal * pose;
    Axis liftAxis (Axis(1,0,0) * pose);
    Axis swingAxis(Axis(0,0,1) * pose);
    UniversalJoint* hip = new UniversalJoint(trunk, leg, anchor, liftAxis, swingAxis);
    hip->init(odeHandle);
    joints.push_back(hip);
    hips.emplace_back(hip, -liftRange, liftRange, -swingRange, swingRange, power, maxVel);

    // foot-ground contact sensing (binary, per leg)
    addSensor(new ContactSensor(leg));
  }
}

void Hexapod::setMotorsIntern(const double* m, int n){
  for (int i = 0; i < 2*NLEG && i < n; i++) target[i] = m[i];
}

void Hexapod::doInternalStuffIntern(GlobalData& g){
  for (int n = 0; n < NLEG; n++){
    hips[n].set(target[2*n], target[2*n+1]);   // [lift, swing]
    hips[n].act(g);
  }
}

int Hexapod::getSensorsIntern(double* s, int n){
  int w = 0;
  for (int i = 0; i < NLEG && w < n; i++){
    if (w < n) s[w++] = hips[i].get1();   // lift angle
    if (w < n) s[w++] = hips[i].get2();   // swing angle
  }
  return w;
}

} // namespace mor
