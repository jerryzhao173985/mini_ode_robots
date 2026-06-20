/* mini_ode_robots — angularmotor.h
 *
 * A general ODE angular motor (dAMotor) — extracted from lpzrobots motors/angularmotor.
 * Unlike a joint's built-in motor, an AMotor can ACTIVELY drive a BallJoint (3-DOF) or
 * arbitrary user axes between two bodies, which the engine otherwise can't actuate.
 * It is a Motor (attach with OdeRobot::addMotor): each command in [-1,1] sets the target
 * angular velocity (×maxVel) of an axis, applied through the joint's implicit motor
 * (dParamVel/FMax) — stable like the velocity servo.
 *
 *   // active 3-DOF control of a ball joint between `upper` and `lower`:
 *   addMotor(new AngularMotor(oh, upper, lower, { {1,0,0},{0,1,0},{0,0,1} }, 20));  // power=20
 */
#ifndef MOR_ANGULARMOTOR_H
#define MOR_ANGULARMOTOR_H

#include "motor.h"
#include "odehandle.h"
#include "primitive.h"
#include "pose_types.h"
#include <vector>
#include <algorithm>

namespace mor {

class AngularMotor : public Motor {
public:
  /** @param axes 1..3 drive axes; @param relative 0=global, 1=body1, 2=body2 frame
      @param power per-axis torque budget (dParamFMax); @param maxVel rad/s at command=1 */
  AngularMotor(const OdeHandle& oh, Primitive* p1, Primitive* p2,
               const std::vector<Axis>& axes, double power, double maxVel = 10.0, int relative = 1)
    : power(power), maxVel(maxVel) {
    naxes = std::min(3, (int)axes.size());
    motor = dJointCreateAMotor(oh.world, 0);
    dJointAttach(motor, p1 ? p1->getBody() : 0, p2 ? p2->getBody() : 0);
    dJointSetAMotorMode(motor, dAMotorUser);
    dJointSetAMotorNumAxes(motor, naxes);
    for (int i=0;i<naxes;i++)
      dJointSetAMotorAxis(motor, i, relative, axes[i].x(), axes[i].y(), axes[i].z());
    for (int i=0;i<3;i++) target[i] = 0;
  }
  ~AngularMotor() override { if (odeIsAlive() && motor) dJointDestroy(motor); }
  AngularMotor(const AngularMotor&) = delete;
  AngularMotor& operator=(const AngularMotor&) = delete;

  /* --- Motor interface --- */
  int  getMotorNumber() const override { return naxes; }
  void set(const double* m, int n) override {
    for (int i=0;i<naxes && i<n;i++) target[i] = std::max(-1.0, std::min(1.0, m[i]));
  }
  void act(const GlobalData&) override {
    if (dBodyID b = dJointGetBody(motor,0)) dBodyEnable(b);  // a sleeping body ignores the motor
    if (dBodyID b = dJointGetBody(motor,1)) dBodyEnable(b);
    for (int i=0;i<naxes;i++) {
      dJointSetAMotorParam(motor, velParam(i),  target[i] * maxVel);
      dJointSetAMotorParam(motor, fmaxParam(i), power);
    }
  }
  /// angular rate about axis i (rad/s)
  double getAngleRate(int i) const { return dJointGetAMotorAngleRate(motor, i); }
  dJointID getJoint() const { return motor; }

private:
  static int velParam (int i){ return i==0 ? dParamVel  : i==1 ? dParamVel2  : dParamVel3;  }
  static int fmaxParam(int i){ return i==0 ? dParamFMax : i==1 ? dParamFMax2 : dParamFMax3; }
  dJointID motor;
  int      naxes;
  double   power, maxVel;
  double   target[3];
};

} // namespace mor
#endif
