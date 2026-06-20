/* mini_ode_robots — hexapod.h
 *
 * A six-legged walker — the engine's capstone robot, distilled from lpzrobots
 * robots/hexapod. A box trunk carries 6 legs (3 per side); each leg is a capsule
 * on a 2-DOF universal HIP (lift + swing) driven by a TwoAxisServoVel, with a foot
 * ContactSensor. Equal-and-opposite tripods give a statically-stable walking gait.
 *
 * Controller I/O: 12 motor channels = [lift,swing] x 6 legs; sensors = 12 hip angles
 * (intern) + 6 foot contacts (attached). The tripod gait lives in the controller
 * (see examples/demo_hexapod.cpp) so the robot itself stays generic.
 *
 * (Simplifications vs lpzrobots, by design — not overkill: 2-DOF legs, no knee/tarsus
 *  spring/IR. The knee is a natural extension — add a HingeJoint+OneAxisServoVel.)
 */
#ifndef MOR_HEXAPOD_H
#define MOR_HEXAPOD_H

#include "mor/oderobot.h"
#include "mor/servo.h"
#include <vector>

namespace mor {

class Hexapod : public OdeRobot {
public:
  static const int NLEG = 6;

  explicit Hexapod(const OdeHandle& odeHandle, const std::string& name = "Hexapod");

  int  getSensorNumberIntern() override { return 2*NLEG; }   // [lift,swing] angle per leg
  int  getMotorNumberIntern()  override { return 2*NLEG; }   // [lift,swing] command per leg
  int  getSensorsIntern(double* s, int n) override;
  void setMotorsIntern(const double* m, int n) override;
  void doInternalStuffIntern(GlobalData& g) override;        // drives the hip servos
  void placeIntern(const Pose& pose) override;

  /// which tripod a leg belongs to (0 or 1) — for writing a tripod gait
  static int tripodOf(int leg){ return ((leg/2) + (leg%2)) % 2; }
  /// natural standing height (place at z = restHeight())
  double restHeight() const { return standH; }

private:
  // geometry (m) / dynamics (kg)
  double bodyL = 0.70, bodyW = 0.30, bodyH = 0.12, bodyMass = 1.2;
  double legR  = 0.035, legLen = 0.45, legMass = 0.12;
  double standH = 0.30;
  double liftRange = 0.6, swingRange = 0.6;   // hip travel (rad)
  double power = 10.0, maxVel = 14.0;          // servo force budget / speed
  std::vector<TwoAxisServoVel> hips;           // one per leg
  double target[2*NLEG] = {0};                 // [lift,swing] per leg
};

} // namespace mor
#endif
