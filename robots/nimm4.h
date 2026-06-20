/* mini_ode_robots — nimm4.h
 *
 * A clean differential-drive (skid-steer) robot in the spirit of lpzrobots'
 * iconic two-wheeled "Nimm2". A box chassis + four cylinder wheels on hinge
 * joints; the left pair and right pair are driven independently (2 motor
 * channels), so equal speeds drive straight and a speed difference turns.
 *
 * It demonstrates the whole engine end to end: Primitive (Box, Cylinder),
 * HingeJoint, ODE servo motors (dParamVel/dParamFMax), joint-rate sensors,
 * per-part Substance (rubber wheels), and the place(pose) build pattern.
 */
#ifndef MOR_NIMM4_H
#define MOR_NIMM4_H

#include "mor/oderobot.h"

namespace mor {

class Nimm4 : public OdeRobot {
public:
  explicit Nimm4(const OdeHandle& odeHandle, const std::string& name = "Nimm4");

  int  getSensorNumberIntern() const override { return 2; }   ///< [left wheel rate, right wheel rate]
  int  getMotorNumberIntern()  const override { return 2; }   ///< [left drive, right drive] in [-1,1]
  int  getSensorsIntern(double* sensors, int n) const override;
  void setMotorsIntern(const double* motors, int n) override;

  void placeIntern(const Pose& pose) override;

  /// natural resting height (place at z = restHeight() so the wheels touch ground)
  double restHeight() const { return wheelRadius; }

private:
  // geometry (metres) / dynamics (kg)
  double length    = 0.50, width = 0.30, cheight = 0.10, chassisMass = 1.0;
  double wheelRadius = 0.15, wheelWidth = 0.05, wheelMass = 0.3;
  double maxSpeed  = 8.0;   ///< rad/s at motor command = 1
  double maxForce  = 4.0;   ///< dParamFMax (torque budget) per wheel
  // joints 0,2 = left (FL,BL); 1,3 = right (FR,BR)
};

} // namespace mor
#endif
