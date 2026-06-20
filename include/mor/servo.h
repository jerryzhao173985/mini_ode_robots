/* mini_ode_robots — servo.h
 *
 * A position-control servo wrapping a OneAxisJoint (Hinge or Slider), extracted
 * from lpzrobots motors/oneaxisservo. set(pos in [-1,1]) drives the joint toward
 * a scaled target angle/position using a PID that emits a torque/force via
 * addForce1, with joint limit-stops and a velocity clamp for stability.
 *
 * This is the reusable CONTROL element legged/arm robots need (position control),
 * built purely on ODE joints + the extracted PID — no selforg dependency.
 *
 * STABILITY NOTE: this is a faithful explicit-PID position servo (torque =
 * KP*(P+I+D), clamped to +/-KP). Because it applies a TORQUE and lets ODE
 * integrate it, it is only stable when `power` (KP) is modest relative to joint
 * inertia and 1/dt: at high gain it limit-cycles and leaves a small steady-state
 * error (a fundamental property of explicit PID against a stiff plant).
 * ===> For position control, prefer `OneAxisServoVel` below: it commands a
 * VELOCITY to the joint's implicit motor and is unconditionally stable with
 * ~zero steady-state error at any gain (this is lpzrobots' default for legged
 * robots, and what `Arm` uses). `OneAxisServo` is kept for fidelity/compliant use.
 */
#ifndef MOR_SERVO_H
#define MOR_SERVO_H

#include "joint.h"
#include "pid.h"
#include "motor.h"
#include <algorithm>
#include <cmath>

namespace mor {

class OneAxisServo : public Motor {
public:
  /** @param joint the hinge/slider to control
      @param min,max travel bounds (min should be <= 0 <= max)
      @param power   PID KP (servo strength)
      @param damp    PID KD; integration -> PID KI
      @param maxVel  linear-velocity clamp on the two parts (0 = off) */
  OneAxisServo(OneAxisJoint* joint, double min, double max,
               double power, double damp = 0.2, double integration = 2.0,
               double maxVel = 10.0, double jointLimit = 1.3)
    : joint(joint), min(min), max(max), pid(power, integration, damp),
      maxVel(maxVel), jointLimit(jointLimit) {
    assert(joint && min < max);
    assert(power >= 0 && damp >= 0 && integration >= 0);
    setMinMax(min, max);
  }

  /// set the target position, pos in [-1,1] mapped into [min,max]
  void set(double pos){
    pos = std::max(-1.0, std::min(1.0, pos));
    pos = (pos > 0) ? pos * max : pos * -min;
    pid.setTargetPosition(pos);
    double force = pid.step(joint->getPosition1(), joint->odeHandle.getTime());
    force = std::min(pid.KP, std::max(-pid.KP, force));  // limit |force| to KP
    joint->addForce1(force);
    if (maxVel > 0){
      joint->getPart1()->limitLinearVel(maxVel);
      joint->getPart2()->limitLinearVel(maxVel);
    }
  }

  /// current joint position scaled to [-1,1] (clamped; may exceed travel at limit stops)
  double get() const {
    double p = joint->getPosition1();
    double s = (p > 0) ? ((max != 0) ? p / max : 0.0) : ((min != 0) ? p / -min : 0.0);
    return std::max(-1.0, std::min(1.0, s));
  }

  void setMinMax(double mn, double mx){
    min = mn; max = mx;
    joint->setParam(dParamLoStop, min - std::fabs(min) * (jointLimit - 1));
    joint->setParam(dParamHiStop, max + std::fabs(max) * (jointLimit - 1));
  }
  void setPower(double power) { pid.KP = power; }
  double getPower() const { return pid.KP; }

  /* --- Motor interface (so a servo can be attached via OdeRobot::addMotor) --- */
  int  getMotorNumber() const override { return 1; }
  void set(const double* m, int n) override { if (n>0) target_ = m[0]; }
  void act(const GlobalData&) override { set(target_); }   // applies via set(double) each step

protected:
  OneAxisJoint* joint;
  double min, max;
  PID pid;
  double maxVel;
  double jointLimit;
  double target_ = 0;
};

typedef OneAxisServo HingeServo;
typedef OneAxisServo SliderServo;

/** RECOMMENDED position controller: a VELOCITY servo (lpzrobots OneAxisServoVel).
 *
 * Instead of applying a torque, it computes a target *velocity* from the position
 * error (a P controller, velocity-limited so it can't overshoot in one step) and
 * feeds it to the joint's built-in motor via dParamVel, with a force budget
 * (dParamFMax) ramped by distance-to-target. ODE solves the velocity constraint
 * IMPLICITLY in the LCP each step, so this is unconditionally stable regardless of
 * `power` — no limit cycle, and it settles to the commanded angle with ~zero
 * steady-state error (verified in test_features). Prefer this over OneAxisServo
 * for position control; it is what lpzrobots uses for its legged robots.
 *
 * (lpzrobots routes this through a separate dAMotor; for a hinge/slider the joint's
 *  own motor is equivalent and simpler, so we use that.)
 */
class OneAxisServoVel : public Motor {
public:
  OneAxisServoVel(OneAxisJoint* joint, double min, double max,
                  double power, double damp = 0.2, double maxVel = 20.0,
                  double jointLimit = 1.3)
    : joint(joint), min(min), max(max), pid(maxVel/2, 0.0, 0.0),
      power(power), damp(std::max(0.0, std::min(1.0, damp))), jointLimit(jointLimit) {
    assert(joint && min < max && power >= 0);
    joint->setParam(dParamLoStop, min - std::fabs(min) * (jointLimit - 1));
    joint->setParam(dParamHiStop, max + std::fabs(max) * (jointLimit - 1));
  }

  /// command target position, pos in [-1,1] mapped (centered) into [min,max]
  void set(double pos){
    pos = std::max(-1.0, std::min(1.0, pos));
    pos = (pos + 1) * (max - min) / 2 + min;            // centered scaling
    pid.setTargetPosition(pos);
    double vel = pid.stepVelocity(joint->getPosition1(), joint->odeHandle.getTime());
    double e   = std::fabs(2.0 * pid.error / (max - min));   // normalized distance to target
    joint->setParam(dParamVel, vel);
    joint->setParam(dParamFMax, std::tanh(e + damp) * power);  // ramp force budget near target
  }

  /// current position scaled to [-1,1] (inverse of the centered set() scaling, clamped)
  double get() const {
    double s = 2.0 * (joint->getPosition1() - min) / (max - min) - 1.0;
    return std::max(-1.0, std::min(1.0, s));
  }
  void setPower(double p){ power = p; }
  double getPower() const { return power; }

  /* --- Motor interface (so a servo can be attached via OdeRobot::addMotor) --- */
  int  getMotorNumber() const override { return 1; }
  void set(const double* m, int n) override { if (n>0) target_ = m[0]; }
  void act(const GlobalData&) override { set(target_); }   // applies via set(double) each step

protected:
  OneAxisJoint* joint;
  double min, max;
  PID    pid;
  double power, damp, jointLimit;
  double target_ = 0;
};

typedef OneAxisServoVel HingeServoVel;

/** Velocity servo for a TWO-axis joint (UniversalJoint) — position control of both
 * axes via their implicit motors (dParamVel/Vel2 + FMax/FMax2). This is what a hexapod
 * hip needs (lift + swing), and what lpzrobots' TwoAxisServoVel provides. */
class TwoAxisServoVel : public Motor {
public:
  TwoAxisServoVel(TwoAxisJoint* joint, double min1, double max1, double min2, double max2,
                  double power, double maxVel = 12.0, double jointLimit = 1.3)
    : joint(joint), min1(min1), max1(max1), min2(min2), max2(max2),
      pid1(maxVel/2, 0, 0), pid2(maxVel/2, 0, 0), power(power) {
    assert(joint && min1 < max1 && min2 < max2 && power >= 0);
    joint->setParam(dParamLoStop,  min1 - std::fabs(min1)*(jointLimit-1));
    joint->setParam(dParamHiStop,  max1 + std::fabs(max1)*(jointLimit-1));
    joint->setParam(dParamLoStop2, min2 - std::fabs(min2)*(jointLimit-1));
    joint->setParam(dParamHiStop2, max2 + std::fabs(max2)*(jointLimit-1));
  }
  /// command both axes, each in [-1,1] (centered-scaled into [min,max])
  void set(double p1, double p2){
    t1 = std::max(-1.0,std::min(1.0,p1)); t2 = std::max(-1.0,std::min(1.0,p2));
  }
  double get1() const { return 2*(joint->getPosition1()-min1)/(max1-min1)-1; }
  double get2() const { return 2*(joint->getPosition2()-min2)/(max2-min2)-1; }

  /* --- Motor interface --- */
  int  getMotorNumber() const override { return 2; }
  void set(const double* m, int n) override { if(n>0) set(m[0], n>1?m[1]:t2); }
  void act(const GlobalData&) override {
    double time = joint->odeHandle.getTime();
    pid1.setTargetPosition((t1+1)*(max1-min1)/2+min1);
    joint->setParam(dParamVel,  pid1.stepVelocity(joint->getPosition1(), time));
    joint->setParam(dParamFMax, power);
    pid2.setTargetPosition((t2+1)*(max2-min2)/2+min2);
    joint->setParam(dParamVel2,  pid2.stepVelocity(joint->getPosition2(), time));
    joint->setParam(dParamFMax2, power);
  }
private:
  TwoAxisJoint* joint;
  double min1,max1,min2,max2;
  PID pid1, pid2;
  double power;
  double t1=0, t2=0;
};

} // namespace mor
#endif
