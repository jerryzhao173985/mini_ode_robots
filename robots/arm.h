/* mini_ode_robots — arm.h
 *
 * A servo-controlled robotic arm: a fixed base + N capsule links joined by
 * hinge joints, each driven by a HingeServo (PID position control). It is the
 * canonical demonstrator of: HingeJoint + OneAxisServo + a multi-joint chain.
 *
 * Controller interface: setMotors(targets[N]) commands each joint to an angle
 * in [-1,1] (scaled to its travel); getSensors() returns the N joint angles.
 */
#ifndef MOR_ARM_H
#define MOR_ARM_H

#include "mor/oderobot.h"
#include "mor/servo.h"
#include <vector>

namespace mor {

class Arm : public OdeRobot {
public:
  Arm(const OdeHandle& odeHandle, int numSegments = 3, const std::string& name = "Arm");

  int  getSensorNumberIntern() override { return nSeg; }
  int  getMotorNumberIntern()  override { return nSeg; }
  int  getSensorsIntern(double* sensors, int n) override;
  void setMotorsIntern(const double* motors, int n) override;
  void doInternalStuffIntern(GlobalData& g) override;   // re-applies the servos each step

  void placeIntern(const Pose& pose) override;
  /// objects[0] is the STATIC base; track the moving tip instead
  Primitive* getMainPrimitive() const override { return objects.empty()? nullptr : objects.back(); }

private:
  int    nSeg;
  double segLen = 0.4, segRad = 0.05, segMass = 0.2;
  double power  = 30.0;            // servo force budget (dParamFMax)
  std::vector<OneAxisServoVel> servos;   // velocity servo: stable position control
  std::vector<double>       targets;
};

} // namespace mor
#endif
