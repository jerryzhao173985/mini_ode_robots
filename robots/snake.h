/* mini_ode_robots — snake.h
 *
 * A planar snake: N capsule segments in a line, joined by vertical-axis hinge
 * joints, each segment given ANISOTROPIC friction (low along the body axis,
 * high laterally — i.e. scales). A lateral undulation (traveling sine wave on
 * the joint angles) then produces net forward locomotion.
 *
 * This is the canonical demonstrator of Substance::toAnisotropFriction — the
 * one extracted feature that no other robot exercises — together with a
 * multi-segment hinge chain and per-joint servos.
 *
 * Controller: getMotorNumber()==N-1 joint targets in [-1,1]; the demo feeds a
 * traveling wave. getSensors() returns the N-1 joint angles.
 */
#ifndef MOR_SNAKE_H
#define MOR_SNAKE_H

#include "mor/oderobot.h"
#include "mor/servo.h"
#include <vector>

namespace mor {

class Snake : public OdeRobot {
public:
  Snake(const OdeHandle& odeHandle, int numSegments = 6, const std::string& name = "Snake");

  int  getSensorNumberIntern() override { return nSeg - 1; }
  int  getMotorNumberIntern()  override { return nSeg - 1; }
  int  getSensorsIntern(double* sensors, int n) override;
  void setMotorsIntern(const double* motors, int n) override;
  void doInternalStuffIntern(GlobalData& g) override;

  void placeIntern(const Pose& pose) override;

  double bodyRadius() const { return segRad; }

private:
  int    nSeg;
  double segLen = 0.30, segRad = 0.06, segMass = 0.2;
  double power  = 8.0;
  double frictionRatio = 0.3;   ///< forward friction / lateral friction (<1 = snake-like)
  std::vector<OneAxisServo> servos;
  std::vector<double>       targets;
};

} // namespace mor
#endif
