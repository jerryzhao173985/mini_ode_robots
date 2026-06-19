/* mini_ode_robots — oderobot.h
 *
 * A robot is a bag of Primitives + Joints plus a controller-facing I/O interface.
 * Extracted from lpzrobots robots/oderobot.{h,cpp} (selforg AbstractRobot/Storeable
 * dropped). The lpzrobots Sensor/Motor ATTACHMENT system is kept: the public
 * getSensors/setMotors aggregate the robot's own (*Intern) channels with any number
 * of attached Sensor/Motor objects, so robots compose sensing/actuation without
 * rewriting their I/O. A concrete robot implements the *Intern hooks (and/or attaches).
 */
#ifndef MOR_ODEROBOT_H
#define MOR_ODEROBOT_H

#include <vector>
#include <string>
#include "odehandle.h"
#include "globaldata.h"
#include "pose_types.h"
#include "sensor.h"
#include "motor.h"

namespace mor {

class Primitive;
class Joint;

class OdeRobot {
public:
  OdeRobot(const OdeHandle& odeHandle, const std::string& name);
  virtual ~OdeRobot();

  /* ---- controller interface: robot's own (*Intern) channels THEN attached sensors/motors ---- */
  int  getSensorNumber();
  int  getMotorNumber();
  int  getSensors(double* sensors, int sensornumber);
  void setMotors(const double* motors, int motornumber);

  /* ---- subclass hooks (a robot implements these; defaults = no own channels) ---- */
  virtual int  getSensorNumberIntern() { return 0; }
  virtual int  getMotorNumberIntern()  { return 0; }
  virtual int  getSensorsIntern(double* sensors, int sensornumber) { (void)sensors; (void)sensornumber; return 0; }
  virtual void setMotorsIntern(const double* motors, int motornumber) { (void)motors; (void)motornumber; }
  virtual void senseIntern(GlobalData& global) { (void)global; }
  virtual void doInternalStuffIntern(GlobalData& global) { (void)global; }

  /* ---- attach generic sensors/motors (the robot takes ownership) ---- */
  void addSensor(Sensor* s);   ///< appended after the robot's own sensor channels
  void addMotor(Motor* m);     ///< appended after the robot's own motor channels

  /* ---- per-step hooks (called by Simulation::run): attached + *Intern ---- */
  void sense(GlobalData& global);            ///< senses attached sensors, then senseIntern
  void doInternalStuff(GlobalData& global);  ///< acts attached motors, then doInternalStuffIntern

  /* ---- placement ---- */
  void place(const Pos& pos);
  void place(const Pose& pose);
  virtual void placeIntern(const Pose& pose) = 0;   ///< subclass builds its parts here

  /* ---- introspection / tracking ---- */
  /// primitive used for tracking/position; defaults to the first part. Override in
  /// robots whose objects[0] is not the body to track (e.g. a static base — see Arm).
  virtual Primitive* getMainPrimitive() const { return objects.empty()? nullptr : objects[0]; }
  const std::vector<Primitive*>& getAllPrimitives() const { return objects; }
  const std::vector<Joint*>&     getAllJoints()     const { return joints; }
  Pos getPosition() const;
  Pos getSpeed() const;
  const std::string& getName() const { return name; }

protected:
  virtual void cleanup();   ///< delete joints then primitives (called by destructor)

  OdeHandle               odeHandle;
  std::string             name;
  std::vector<Primitive*> objects;   ///< populated by subclasses in placeIntern
  std::vector<Joint*>     joints;     ///< populated by subclasses in placeIntern
  std::vector<Sensor*>    attachedSensors;  ///< owned; deleted in cleanup()
  std::vector<Motor*>     attachedMotors;   ///< owned; deleted in cleanup()
  Pose                    initialPose;
};

} // namespace mor
#endif
