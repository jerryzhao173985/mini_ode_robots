/* mini_ode_robots — joint.h
 *
 * A Joint constrains two Primitives. Every subclass is a thin wrapper over the
 * matching dJointCreate* / dJointAttach / set-anchor-and-axis calls, plus a
 * uniform actuation+sensing interface (addForce / getPosition / getPositionRate
 * / setParam). Extracted from lpzrobots osg/joint.{h,cpp}; visuals removed.
 *
 * The DOF-based hierarchy mirrors the joints' kinematics:
 *   Joint            (0 axes: Fixed, Ball)
 *    └ OneAxisJoint  (1 axis : Hinge, Slider)
 *       └ TwoAxisJoint (2 axes: Hinge2, Universal)
 */
#ifndef MOR_JOINT_H
#define MOR_JOINT_H

#include <ode/ode.h>
#include <cassert>
#include <list>
#include "pose_types.h"
#include "odehandle.h"
#include "primitive.h"

namespace mor {

class Joint {
public:
  Joint(Primitive* part1, Primitive* part2, const Vec3& anchor)
    : joint(0), part1(part1), part2(part2), anchor(anchor), feedback(0) {
    assert(part1 && part2);
  }
  virtual ~Joint();
  // Owns a raw dJointID and a malloc'd feedback block; copying would double-free.
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;

  /** Create the joint. If ignoreColl, the two parts' geoms are marked an
      ignored pair so they don't collide (the usual case). */
  virtual void init(const OdeHandle& odeHandle, bool ignoreColl = true);

  virtual void setParam(int parameter, double value) = 0;
  virtual double getParam(int parameter) const = 0;

  dJointID getJoint() const { return joint; }
  Primitive* getPart1() { return part1; }
  Primitive* getPart2() { return part2; }
  Vec3 getAnchor() const { return anchor; }

  virtual int  getNumberAxes() const = 0;
  virtual Axis getAxis(int n) const { (void)n; return Axis(); }

  virtual std::list<double> getPositions() const { return {}; }
  virtual std::list<double> getPositionRates() const { return {}; }
  virtual int getPositions(double* a) const { (void)a; return 0; }
  virtual int getPositionRates(double* a) const { (void)a; return 0; }

  /* constraint force/torque feedback */
  void setFeedBackMode(bool mode);
  bool getTorqueFeedback(Pos& t1, Pos& t2) const;
  bool getForceFeedback(Pos& f1, Pos& f2) const;

protected:
  dJointID   joint;
  Primitive* part1;
  Primitive* part2;
  Vec3       anchor;
  dJointFeedback* feedback;
public:
  OdeHandle odeHandle;
};

class OneAxisJoint : public Joint {
public:
  OneAxisJoint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& axis1)
    : Joint(p1,p2,anchor), axis1(axis1) {}
  Axis getAxis(int) const override { return axis1; }
  Axis getAxis1() const { return axis1; }
  virtual double getPosition1() const = 0;
  virtual double getPosition1Rate() const = 0;
  virtual void   addForce1(double force) = 0;
  int getNumberAxes() const override { return 1; }
  std::list<double> getPositions() const override { return { getPosition1() }; }
  std::list<double> getPositionRates() const override { return { getPosition1Rate() }; }
  int getPositions(double* a) const override { a[0]=getPosition1(); return 1; }
  int getPositionRates(double* a) const override { a[0]=getPosition1Rate(); return 1; }
protected:
  Axis axis1;
};

class TwoAxisJoint : public OneAxisJoint {
public:
  TwoAxisJoint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& a1, const Axis& a2)
    : OneAxisJoint(p1,p2,anchor,a1), axis2(a2) {}
  Axis getAxis(int n) const override { return n==0?axis1:axis2; }
  Axis getAxis2() const { return axis2; }
  virtual double getPosition2() const = 0;
  virtual double getPosition2Rate() const = 0;
  virtual void   addForce2(double force) = 0;
  void addForces(double f1,double f2){ addForce1(f1); addForce2(f2); }
  int getNumberAxes() const override { return 2; }
  std::list<double> getPositions() const override { return { getPosition1(), getPosition2() }; }
  std::list<double> getPositionRates() const override { return { getPosition1Rate(), getPosition2Rate() }; }
  int getPositions(double* a) const override { a[0]=getPosition1(); a[1]=getPosition2(); return 2; }
  int getPositionRates(double* a) const override { a[0]=getPosition1Rate(); a[1]=getPosition2Rate(); return 2; }
protected:
  Axis axis2;
};

/* ----------------------------------------------------- concrete joints */

class FixedJoint : public Joint {
public:
  FixedJoint(Primitive* p1, Primitive* p2, const Vec3& anchor = Vec3(0,0,0));
  void init(const OdeHandle&, bool ignoreColl=true) override;
  void setParam(int, double) override {}
  double getParam(int) const override { return 0; }
  int getNumberAxes() const override { return 0; }
};

class HingeJoint : public OneAxisJoint {
public:
  HingeJoint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& axis1);
  void init(const OdeHandle&, bool ignoreColl=true) override;
  void   addForce1(double t) override;
  double getPosition1() const override;
  double getPosition1Rate() const override;
  void   setParam(int parameter, double value) override;
  double getParam(int parameter) const override;
};

class Hinge2Joint : public TwoAxisJoint {
public:
  Hinge2Joint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& a1, const Axis& a2);
  void init(const OdeHandle&, bool ignoreColl=true) override;
  void   addForce1(double t1) override;
  void   addForce2(double t2) override;
  double getPosition1() const override;
  double getPosition2() const override;       ///< NOT supported by ODE hinge2; warns
  double getPosition1Rate() const override;
  double getPosition2Rate() const override;
  void   setParam(int parameter, double value) override;
  double getParam(int parameter) const override;
};

class UniversalJoint : public TwoAxisJoint {
public:
  UniversalJoint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& a1, const Axis& a2);
  void init(const OdeHandle&, bool ignoreColl=true) override;
  void   addForce1(double t1) override;
  void   addForce2(double t2) override;
  double getPosition1() const override;
  double getPosition2() const override;
  double getPosition1Rate() const override;
  double getPosition2Rate() const override;
  void   setParam(int parameter, double value) override;
  double getParam(int parameter) const override;
};

class BallJoint : public Joint {
public:
  BallJoint(Primitive* p1, Primitive* p2, const Vec3& anchor);
  void init(const OdeHandle&, bool ignoreColl=true) override;
  void setParam(int, double) override {}
  double getParam(int) const override { return 0; }
  int getNumberAxes() const override { return 0; }
};

class SliderJoint : public OneAxisJoint {
public:
  SliderJoint(Primitive* p1, Primitive* p2, const Vec3& anchor, const Axis& axis1);
  void init(const OdeHandle&, bool ignoreColl=true) override;
  void   addForce1(double t) override;
  double getPosition1() const override;
  double getPosition1Rate() const override;
  void   setParam(int parameter, double value) override;
  double getParam(int parameter) const override;
};

} // namespace mor
#endif
