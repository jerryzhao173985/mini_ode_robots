/* mini_ode_robots — joint.cpp
 * Extracted from lpzrobots osg/joint.cpp (OSG visuals removed).
 */
#include "mor/joint.h"
#include <cstdlib>
#include <cstdio>

namespace mor {

/* ------------------------------------------------------------- Joint base */
Joint::~Joint(){
  if (!odeIsAlive()) return;   // ODE world gone (wrong-order teardown) — skip safely
  setFeedBackMode(false);
  if (joint) dJointDestroy(joint);
  if (part1 && part2 && part1->getGeom() && part2->getGeom())
    odeHandle.removeIgnoredPair(part1->getGeom(), part2->getGeom());
}

void Joint::init(const OdeHandle& oh, bool ignoreColl){
  this->odeHandle = oh;
  assert(part1->getBody() && "only part2 may be static (body 0)");
  if (ignoreColl && part1->getGeom() && part2->getGeom())
    this->odeHandle.addIgnoredPair(part1->getGeom(), part2->getGeom());
}

void Joint::setFeedBackMode(bool mode){
  if (mode){
    if ((feedback = dJointGetFeedback(joint))==0){
      feedback = (dJointFeedback*)malloc(sizeof(dJointFeedback));
      dJointSetFeedback(joint, feedback);
    }
  } else if (feedback){
    dJointSetFeedback(joint, 0);
    free(feedback);
    feedback = 0;
  }
}
bool Joint::getTorqueFeedback(Pos& t1, Pos& t2) const {
  dJointFeedback* fb = dJointGetFeedback(joint);
  if (!fb) return false;
  t1 = Pos(fb->t1); t2 = Pos(fb->t2); return true;
}
bool Joint::getForceFeedback(Pos& f1, Pos& f2) const {
  dJointFeedback* fb = dJointGetFeedback(joint);
  if (!fb) return false;
  f1 = Pos(fb->f1); f2 = Pos(fb->f2); return true;
}

/* ------------------------------------------------------------- Fixed */
FixedJoint::FixedJoint(Primitive* p1, Primitive* p2, const Vec3& anchor)
  : Joint(p1,p2,anchor) {
  if (anchor.x()||anchor.y()||anchor.z()) this->anchor = p1->toLocal(anchor);
}
void FixedJoint::init(const OdeHandle& oh, bool ignoreColl){
  Joint::init(oh, ignoreColl);
  joint = dJointCreateFixed(oh.world, 0);
  dJointAttach(joint, part1->getBody(), part2->getBody());
  dJointSetFixed(joint);
}

/* ------------------------------------------------------------- Hinge */
HingeJoint::HingeJoint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& axis1)
  : OneAxisJoint(p1,p2,anchor,axis1) {}
void HingeJoint::init(const OdeHandle& oh, bool ignoreColl){
  Joint::init(oh, ignoreColl);
  joint = dJointCreateHinge(oh.world, 0);
  dJointAttach(joint, part1->getBody(), part2->getBody());
  dJointSetHingeAnchor(joint, anchor.x(), anchor.y(), anchor.z());
  dJointSetHingeAxis(joint, axis1.x(), axis1.y(), axis1.z());
}
void   HingeJoint::addForce1(double t){ dJointAddHingeTorque(joint, t); }
double HingeJoint::getPosition1() const { return dJointGetHingeAngle(joint); }
double HingeJoint::getPosition1Rate() const { return dJointGetHingeAngleRate(joint); }
void   HingeJoint::setParam(int p, double v){ assert(joint); dJointSetHingeParam(joint,p,v); }
double HingeJoint::getParam(int p) const { assert(joint); return dJointGetHingeParam(joint,p); }

/* ------------------------------------------------------------- Hinge2 */
Hinge2Joint::Hinge2Joint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& a1, const Axis& a2)
  : TwoAxisJoint(p1,p2,anchor,a1,a2) {}
void Hinge2Joint::init(const OdeHandle& oh, bool ignoreColl){
  // axes must be linearly independent or ODE aborts the process in dxNormalize3
  assert(axis1.crossProduct(axis2).vec3().length() > 1e-6 &&
         "Hinge2 axis1 and axis2 must be linearly independent (not parallel)");
  Joint::init(oh, ignoreColl);
  joint = dJointCreateHinge2(oh.world, 0);
  dJointAttach(joint, part1->getBody(), part2->getBody());
  dJointSetHinge2Anchor(joint, anchor.x(), anchor.y(), anchor.z());
  dJointSetHinge2Axis1(joint, axis1.x(), axis1.y(), axis1.z());
  dJointSetHinge2Axis2(joint, axis2.x(), axis2.y(), axis2.z());
}
void   Hinge2Joint::addForce1(double t1){ dJointAddHinge2Torques(joint, t1, 0); }
void   Hinge2Joint::addForce2(double t2){ dJointAddHinge2Torques(joint, 0, t2); }
double Hinge2Joint::getPosition1() const { return dJointGetHinge2Angle1(joint); }
double Hinge2Joint::getPosition2() const {
  // ODE's dJointGetHinge2Angle2 wraps at +-pi and is unusable as an accumulating
  // sensor, so we report 0 (use getPosition2Rate() for axis-2 wheel speed). Warn once.
  static bool warned = false;
  if (!warned){ std::fprintf(stderr, "Hinge2Joint::getPosition2() not usable (ODE wraps angle2); returns 0 — use getPosition2Rate().\n"); warned = true; }
  return 0;
}
double Hinge2Joint::getPosition1Rate() const { return dJointGetHinge2Angle1Rate(joint); }
double Hinge2Joint::getPosition2Rate() const { return dJointGetHinge2Angle2Rate(joint); }
void   Hinge2Joint::setParam(int p, double v){ assert(joint); dJointSetHinge2Param(joint,p,v); }
double Hinge2Joint::getParam(int p) const { assert(joint); return dJointGetHinge2Param(joint,p); }

/* ------------------------------------------------------------- Universal */
UniversalJoint::UniversalJoint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& a1, const Axis& a2)
  : TwoAxisJoint(p1,p2,anchor,a1,a2) {}
void UniversalJoint::init(const OdeHandle& oh, bool ignoreColl){
  // axes must be linearly independent or ODE aborts the process in dxNormalize3
  assert(axis1.crossProduct(axis2).vec3().length() > 1e-6 &&
         "Universal axis1 and axis2 must be linearly independent (not parallel)");
  Joint::init(oh, ignoreColl);
  joint = dJointCreateUniversal(oh.world, 0);
  dJointAttach(joint, part1->getBody(), part2->getBody());
  dJointSetUniversalAnchor(joint, anchor.x(), anchor.y(), anchor.z());
  dJointSetUniversalAxis1(joint, axis1.x(), axis1.y(), axis1.z());
  dJointSetUniversalAxis2(joint, axis2.x(), axis2.y(), axis2.z());
}
void   UniversalJoint::addForce1(double t1){ dJointAddUniversalTorques(joint, t1, 0); }
void   UniversalJoint::addForce2(double t2){ dJointAddUniversalTorques(joint, 0, t2); }
double UniversalJoint::getPosition1() const { return dJointGetUniversalAngle1(joint); }
double UniversalJoint::getPosition2() const { return dJointGetUniversalAngle2(joint); }
double UniversalJoint::getPosition1Rate() const { return dJointGetUniversalAngle1Rate(joint); }
double UniversalJoint::getPosition2Rate() const { return dJointGetUniversalAngle2Rate(joint); }
void   UniversalJoint::setParam(int p, double v){ dJointSetUniversalParam(joint,p,v); }
double UniversalJoint::getParam(int p) const { return dJointGetUniversalParam(joint,p); }

/* ------------------------------------------------------------- Ball */
BallJoint::BallJoint(Primitive* p1, Primitive* p2, const Vec3& anchor)
  : Joint(p1,p2,anchor) {}
void BallJoint::init(const OdeHandle& oh, bool ignoreColl){
  Joint::init(oh, ignoreColl);
  joint = dJointCreateBall(oh.world, 0);
  dJointAttach(joint, part1->getBody(), part2->getBody());
  dJointSetBallAnchor(joint, anchor.x(), anchor.y(), anchor.z());
}

/* ------------------------------------------------------------- Slider */
SliderJoint::SliderJoint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& axis1)
  : OneAxisJoint(p1,p2,anchor,axis1) {}
void SliderJoint::init(const OdeHandle& oh, bool ignoreColl){
  Joint::init(oh, ignoreColl);
  joint = dJointCreateSlider(oh.world, 0);
  dJointAttach(joint, part1->getBody(), part2->getBody());
  dJointSetSliderAxis(joint, axis1.x(), axis1.y(), axis1.z());
}
void   SliderJoint::addForce1(double t){ dJointAddSliderForce(joint, t); }
double SliderJoint::getPosition1() const { return dJointGetSliderPosition(joint); }
double SliderJoint::getPosition1Rate() const { return dJointGetSliderPositionRate(joint); }
void   SliderJoint::setParam(int p, double v){ assert(joint); dJointSetSliderParam(joint,p,v); }
double SliderJoint::getParam(int p) const { assert(joint); return dJointGetSliderParam(joint,p); }

} // namespace mor
