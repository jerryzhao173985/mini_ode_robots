/* mini_ode_robots — primitive.h
 *
 * A Primitive welds ODE's two halves into one object:
 *   - a dBodyID  (mass + motion)        if (mode & Body)
 *   - a dGeomID  (shape + collision)    if (mode & Geom)
 * linked by dGeomSetBody() — "the most important call in the API" (ODE skill rule #1).
 *
 * Extracted from lpzrobots osg/primitive.{h,cpp}. The OSG `Draw` half and all of
 * its dependencies (OSGPrimitive, color, texture, BoundingShape) are removed; the
 * physics (body, geom, mass, force, the geom-transform composite) is preserved 1:1.
 */
#ifndef MOR_PRIMITIVE_H
#define MOR_PRIMITIVE_H

#include <ode/ode.h>
#include <cstdio>
#include "pose_types.h"
#include "substance.h"

namespace mor {

class OdeHandle;

/// Build a Pose from the current ODE state of a geom / body.
Pose poseOfGeom(dGeomID geom);
Pose poseOfBody(dBodyID body);

/// counts joint max-velocity violations globally (a stability diagnostic)
extern int globalNumVelocityViolations;

/** Abstract base: a physical (and optionally collidable) rigid object. */
class Primitive {
public:
  /// Body = has a dBody (dynamics); Geom = has a dGeom (collision);
  /// Density = interpret mass as density; _Child/_Transform are internal.
  enum Modes    { Body=1, Geom=2, Draw=4 /*no-op*/, Density=8, _Child=16, _Transform=32 };
  enum Category { Dyn=1, Stat=2 };

  Primitive();
  virtual ~Primitive();
  // Owns raw ODE handles (geom/body); copying would double-free. Use by pointer.
  Primitive(const Primitive&) = delete;
  Primitive& operator=(const Primitive&) = delete;

  /** Create the ODE body and/or geom. @param mode a conjunction of Modes. */
  virtual void init(const OdeHandle& odeHandle, double mass,
                    char mode = Body | Geom) = 0;

  virtual void setMass(double mass, bool density = false) = 0;
  /// full mass spec: center of gravity + inertia tensor
  void setMass(double mass, double cgx, double cgy, double cgz,
               double I11, double I22, double I33, double I12, double I13, double I23);

  /** Fill `m` with this shape's ODE mass for the given total-mass/density, WITHOUT
      needing a body. Returns false for massless shapes (Ray/Dummy). Used to compose
      the inertia of Transform composite bodies (parallel-axis). */
  virtual bool getShapeMass(dMass& m, double mass, bool density) const {
    (void)m; (void)mass; (void)density; return false;
  }

  void  setPosition(const Pos& pos);
  void  setPose(const Pose& pose);
  Pos   getPosition()   const;
  Pose  getPose()       const;
  Pos   getVel()        const;
  Pos   getAngularVel() const;

  bool  applyForce(const Vec3& force);
  bool  applyForce(double x, double y, double z);
  bool  applyTorque(const Vec3& torque);
  bool  applyTorque(double x, double y, double z);

  dGeomID getGeom() const { return geom; }
  dBodyID getBody() const { return body; }

  bool  limitLinearVel(double maxVel);
  bool  limitAngularVel(double maxVel);
  void  decellerate(double factorLin, double factorAng);

  Vec3  toLocal(const Vec3& pos)  const;   ///< world point  -> local
  Vec4  toLocal(const Vec4& axis) const;
  Vec3  toGlobal(const Vec3& pos) const;   ///< local point  -> world
  Vec4  toGlobal(const Vec4& axis) const;

  void  setSubstance(const Substance& s);
  int   getNumVelocityViolations() const { return numVelocityViolations; }
  static void setDestroyGeomFlag(bool v){ destroyGeom = v; }

  /// binary serialize/restore (pose + linear/angular velocity)
  bool  store(FILE* f) const;
  bool  restore(FILE* f);

  Substance substance;

protected:
  /// dGeomSetBody + category/collide bits + back-pointer (the BRIDGE). assumes mode&Geom.
  void attachGeomAndSetColliderFlags();

  dGeomID geom;
  dBodyID body;
  char    mode;
  bool    substanceManuallySet;
  int     numVelocityViolations;
  static bool destroyGeom;
};

/* -------------------------------------------------------------- shapes */

/** Static ground/wall plane a*x+b*y+c*z = d  (default: z=0 ground). A geom with NO body. */
class Plane : public Primitive {
public:
  Plane(double a=0,double b=0,double c=1,double d=0);
  void init(const OdeHandle&, double mass, char mode = Geom) override;
  void setMass(double mass, bool density=false) override;
private:
  double a,b,c,d;
};

class Box : public Primitive {
public:
  Box(double lx, double ly, double lz);
  explicit Box(const Vec3& dim);
  void init(const OdeHandle&, double mass, char mode = Body|Geom) override;
  void setMass(double mass, bool density=false) override;
  bool getShapeMass(dMass& m, double mass, bool density) const override;
  Vec3 getDim() const { return dim; }
private:
  Vec3 dim;  ///< FULL side lengths (ODE convention, skill rule #11)
};

class Sphere : public Primitive {
public:
  explicit Sphere(double radius);
  void init(const OdeHandle&, double mass, char mode = Body|Geom) override;
  void setMass(double mass, bool density=false) override;
  bool getShapeMass(dMass& m, double mass, bool density) const override;
  double getRadius() const { return radius; }
private:
  double radius;
};

/** Capsule = cylinder with hemispherical caps, long axis = local Z. */
class Capsule : public Primitive {
public:
  Capsule(double radius, double height);
  void init(const OdeHandle&, double mass, char mode = Body|Geom) override;
  void setMass(double mass, bool density=false) override;
  bool getShapeMass(dMass& m, double mass, bool density) const override;
  double getRadius() const { return radius; }
  double getHeight() const { return height; }
private:
  double radius, height;  ///< height EXCLUDES the caps (skill rule #11)
};

class Cylinder : public Primitive {
public:
  Cylinder(double radius, double height);
  void init(const OdeHandle&, double mass, char mode = Body|Geom) override;
  void setMass(double mass, bool density=false) override;
  bool getShapeMass(dMass& m, double mass, bool density) const override;
  double getRadius() const { return radius; }
  double getHeight() const { return height; }
private:
  double radius, height;
};

/** A sensing ray of the given range (long axis = local Z, origin at center). */
class Ray : public Primitive {
public:
  explicit Ray(double range);
  void init(const OdeHandle&, double mass, char mode = Geom) override;
  void setMass(double mass, bool density=false) override {}
  void setLength(double len) { length = len; }
  double getRange() const { return range; }
private:
  double range, length;
};

/** Encapsulated geom: rigidly offsets `child`'s geom relative to `parent`'s body.
    This is how one rigid body gets contributions from several shapes (composite mass). */
class Transform : public Primitive {
public:
  Transform(Primitive* parent, Primitive* child, const Pose& pose, bool deleteChild = true);
  ~Transform() override;
  void init(const OdeHandle&, double mass, char mode = Body|Geom) override;
  void setMass(double mass, bool density=false) override;
  Primitive* getChild() const { return child; }
private:
  Primitive* parent;
  Primitive* child;
  Pose       pose;
  bool       deleteChild;
};

/** No body, no geom — represents the static world or a manually-tracked point. */
class DummyPrimitive : public Primitive {
public:
  DummyPrimitive() { body=0; geom=0; }
  void init(const OdeHandle&, double, char) override {}
  void setMass(double, bool) override {}
};

} // namespace mor
#endif
