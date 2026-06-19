/* mini_ode_robots — pid.h
 * The PID controller used by the servo motor, extracted verbatim from
 * lpzrobots motors/pid.{h,cpp}. KP is a global gain; KI/KD tune independently.
 * step() includes the classic I/D clamps that keep ODE joint servos stable.
 */
#ifndef MOR_PID_H
#define MOR_PID_H

#include <algorithm>
#include <cmath>

namespace mor {

class PID {
public:
  double targetposition = 0;
  double position = 0, lastposition = 0;
  double error = 0, lasterror = 0, derivative = 0;
  double KP, KI, KD;
  double tau = 1000;
  double P = 0, I = 0, D = 0;
  double force = 0;
  double lasttime = -1;

  PID(double KP = 100, double KI = 2.0, double KD = 0.3) : KP(KP), KI(KI), KD(KD) {}

  void setKP(double kp){ KP = kp; }
  void setTargetPosition(double newpos){ targetposition = newpos; }
  double getTargetPosition() const { return targetposition; }

  /// classic PID with I/D cutoffs (lpzrobots PID::step) — output is a force/torque
  double step(double newsensorval, double time){
    if (lasttime != -1 && time - lasttime > 0){
      lastposition = position;
      position = newsensorval;
      double stepsize = time - lasttime;
      lasterror = error;
      error = targetposition - position;
      derivative = (lasterror - error) / stepsize;
      P = error;
      I += stepsize * error * KI;
      I = std::min(0.5, std::max(-0.5, I));     // clamp integral
      D = -derivative * KD;
      D = std::min(0.9, std::max(-0.9, D));     // clamp derivative
      force = KP * (P + I + D);
    } else {
      force = 0;
    }
    lasttime = time;
    return force;
  }

  /// velocity-control variant (lpzrobots PID::stepVelocity): the output is a
  /// NOMINAL VELOCITY (no I term), clamped so it cannot overshoot the error in
  /// one step. Fed to a joint motor (dParamVel) this gives stable position control.
  double stepVelocity(double newsensorval, double time){
    if (lasttime != -1 && time - lasttime > 0){
      lastposition = position;
      position = newsensorval;
      double stepsize = time - lasttime;
      lasterror = error;
      error = targetposition - position;
      P = error;
      if (KD != 0.0){
        derivative += ((lasterror - error) / stepsize - derivative);
        D = -derivative * KD;
        force = KP * (P + D);
      } else {
        force = KP * P;
      }
      if (stepsize * std::fabs(force) > std::fabs(error))  // don't overshoot in one step
        force = error / stepsize;
    } else {
      force = 0;
    }
    lasttime = time;
    return force;
  }
};

} // namespace mor
#endif
