/* mini_ode_robots — oderobot.cpp */
#include "mor/oderobot.h"
#include "mor/primitive.h"
#include "mor/joint.h"

namespace mor {

OdeRobot::OdeRobot(const OdeHandle& odeHandle, const std::string& name)
  : odeHandle(odeHandle), name(name) {}

OdeRobot::~OdeRobot(){ cleanup(); }

void OdeRobot::cleanup(){
  for (Sensor* s : attachedSensors) delete s;   // delete attached I/O first (may free geoms)
  for (Motor* m : attachedMotors)   delete m;
  attachedSensors.clear(); attachedMotors.clear();
  for (Joint* j : joints)      delete j;        // joints before primitives (removes ignored pairs)
  for (Primitive* p : objects) delete p;
  joints.clear();
  objects.clear();
}

/* ---- controller I/O: robot's own (*Intern) channels, then attached sensors/motors ---- */
int OdeRobot::getSensorNumber(){
  int n = getSensorNumberIntern();
  for (Sensor* s : attachedSensors) n += s->getSensorNumber();
  return n;
}
int OdeRobot::getMotorNumber(){
  int n = getMotorNumberIntern();
  for (Motor* m : attachedMotors) n += m->getMotorNumber();
  return n;
}
int OdeRobot::getSensors(double* sensors, int n){
  int w = getSensorsIntern(sensors, n);
  for (Sensor* s : attachedSensors){
    if (w >= n) break;
    w += s->get(sensors + w, n - w);
  }
  return w;
}
void OdeRobot::setMotors(const double* motors, int n){
  setMotorsIntern(motors, n);                // intern reads its own front slice
  int off = getMotorNumberIntern();
  for (Motor* m : attachedMotors){
    if (off >= n) break;
    m->set(motors + off, n - off);
    off += m->getMotorNumber();
  }
}
void OdeRobot::sense(GlobalData& g){
  for (Sensor* s : attachedSensors) s->sense(g);
  senseIntern(g);
}
void OdeRobot::doInternalStuff(GlobalData& g){
  for (Motor* m : attachedMotors) m->act(g);
  doInternalStuffIntern(g);
}
void OdeRobot::addSensor(Sensor* s){ attachedSensors.push_back(s); }
void OdeRobot::addMotor(Motor* m){ attachedMotors.push_back(m); }

void OdeRobot::place(const Pos& pos){ place(Pose::translate(pos)); }
void OdeRobot::place(const Pose& pose){
  initialPose = pose;
  placeIntern(pose);
}

Pos OdeRobot::getPosition() const {
  Primitive* p = getMainPrimitive();
  return p ? p->getPosition() : Pos(0,0,0);
}
Pos OdeRobot::getSpeed() const {
  Primitive* p = getMainPrimitive();
  return p ? p->getVel() : Pos(0,0,0);
}

} // namespace mor
