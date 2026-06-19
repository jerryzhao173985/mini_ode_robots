/* mini_ode_robots — primitive.cpp
 * Extracted from lpzrobots osg/primitive.cpp (OSG/visual code removed).
 */
#include "mor/primitive.h"
#include "mor/odehandle.h"
#include <cassert>
#include <cmath>

namespace mor {

bool Primitive::destroyGeom = true;
int  globalNumVelocityViolations = 0;

Pose poseOfGeom(dGeomID geom){ return poseFromOde(dGeomGetPosition(geom), dGeomGetRotation(geom)); }
Pose poseOfBody(dBodyID body){ return poseFromOde(dBodyGetPosition(body), dBodyGetRotation(body)); }

/* ----------------------------------------------------------- Primitive base */

Primitive::Primitive()
  : geom(0), body(0), mode(0), substanceManuallySet(false), numVelocityViolations(0) {}

Primitive::~Primitive(){
  // Only touch ODE if a world is still alive; otherwise these handles were
  // already freed by Simulation::close() (wrong-order teardown) — skip safely.
  if (!odeIsAlive()) return;
  if (destroyGeom && geom) dGeomDestroy(geom);
  if (body && ((mode & _Transform) == 0)) dBodyDestroy(body);
}

// THE BRIDGE: tie shape to body, set collision-filter bits, store the back-pointer.
void Primitive::attachGeomAndSetColliderFlags(){
  if (mode & Body){
    dGeomSetBody(geom, body);                  // shape now rides the body
    dGeomSetCategoryBits(geom, Dyn);
    dGeomSetCollideBits(geom, ~0x0);           // dynamic: collides with everything
  } else {
    dGeomSetCategoryBits(geom, Stat);
    dGeomSetCollideBits(geom, ~Stat);          // static: not with other statics
  }
  if (mode & _Child){                          // a transform child is always dynamic
    dGeomSetCategoryBits(geom, Dyn);
    dGeomSetCollideBits(geom, ~0x0);
  }
  dGeomSetData(geom, (void*)this);             // geom -> owning Primitive (for the collision callback)
}

void Primitive::setMass(double mass, double cgx,double cgy,double cgz,
                        double I11,double I22,double I33,double I12,double I13,double I23){
  dMass m;
  dMassSetParameters(&m, mass, cgx,cgy,cgz, I11,I22,I33, I12,I13,I23);
  dBodySetMass(body, &m);
}

void Primitive::setPosition(const Pos& pos){
  if (body)      dBodySetPosition(body, pos.x(), pos.y(), pos.z());
  else if (geom) dGeomSetPosition(geom, pos.x(), pos.y(), pos.z());
}

void Primitive::setPose(const Pose& pose){
  Vec3 p = pose.getTrans();
  Quat q; pose.get(q);
  dReal quat[4] = { q.w(), q.x(), q.y(), q.z() };   // (x,y,z,w) -> ODE (w,x,y,z)
  if (body){
    dBodySetPosition(body, p.x(), p.y(), p.z());
    dBodySetQuaternion(body, quat);
  } else if (geom){
    dGeomSetPosition(geom, p.x(), p.y(), p.z());
    dGeomSetQuaternion(geom, quat);
  } else {
    assert(0 && "setPose only after init");
  }
}

Pos  Primitive::getPosition() const {
  if (geom)      return Pos(dGeomGetPosition(geom));
  else if (body) return Pos(dBodyGetPosition(body));
  return Pos(0,0,0);
}
Pose Primitive::getPose() const {
  if (!geom){
    if (!body) return Pose::translate(0,0,0);
    return poseOfBody(body);
  }
  return poseOfGeom(geom);
}
Pos Primitive::getVel()        const { return body ? Pos(dBodyGetLinearVel(body))  : Pos(0,0,0); }
Pos Primitive::getAngularVel() const { return body ? Pos(dBodyGetAngularVel(body)) : Pos(0,0,0); }

bool Primitive::applyForce(const Vec3& f){ return applyForce(f.x(),f.y(),f.z()); }
bool Primitive::applyForce(double x,double y,double z){
  if (!body) return false; dBodyAddForce(body,x,y,z); return true;
}
bool Primitive::applyTorque(const Vec3& t){ return applyTorque(t.x(),t.y(),t.z()); }
bool Primitive::applyTorque(double x,double y,double z){
  if (!body) return false; dBodyAddTorque(body,x,y,z); return true;
}

bool Primitive::limitLinearVel(double maxVel){
  if (!body) return false;
  const dReal* v = dBodyGetLinearVel(body);
  double l2 = v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
  if (l2 > maxVel*maxVel){
    numVelocityViolations++; globalNumVelocityViolations++;
    double s = std::sqrt(l2)/maxVel;
    dBodySetLinearVel(body, v[0]/s, v[1]/s, v[2]/s);
    return true;
  }
  return false;
}
bool Primitive::limitAngularVel(double maxVel){
  if (!body) return false;
  const dReal* v = dBodyGetAngularVel(body);
  double l2 = v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
  if (l2 > maxVel*maxVel){
    numVelocityViolations++; globalNumVelocityViolations++;
    double s = std::sqrt(l2)/maxVel;
    dBodySetAngularVel(body, v[0]/s, v[1]/s, v[2]/s);
    return true;
  }
  return false;
}
void Primitive::decellerate(double factorLin, double factorAng){
  if (!body) return;
  if (factorLin!=0) applyForce(getVel()        * (-factorLin));
  if (factorAng!=0) applyTorque(getAngularVel()* (-factorAng));
}

Vec3 Primitive::toLocal(const Vec3& pos)  const { return pos * Pose::inverse(getPose()); }
Vec4 Primitive::toLocal(const Vec4& axis) const { return axis* Pose::inverse(getPose()); }
Vec3 Primitive::toGlobal(const Vec3& pos) const { return pos * getPose(); }
Vec4 Primitive::toGlobal(const Vec4& axis)const { return axis* getPose(); }

void Primitive::setSubstance(const Substance& s){ substance = s; substanceManuallySet = true; }

bool Primitive::store(FILE* f) const {
  Pose pose = getPose(); Pos vel = getVel(); Pos avel = getAngularVel();
  if (fwrite(pose.ptr(), sizeof(double), 16, f)==16 &&
      fwrite(vel.ptr(),  sizeof(double), 3,  f)==3  &&
      fwrite(avel.ptr(), sizeof(double), 3,  f)==3) return true;
  return false;
}
bool Primitive::restore(FILE* f){
  Pose pose; Pos vel, avel;
  if (fread(pose.ptr(), sizeof(double),16,f)==16 &&
      fread(vel._v,      sizeof(double),3, f)==3  &&
      fread(avel._v,     sizeof(double),3, f)==3){
    setPose(pose);
    if (body){ dBodySetLinearVel(body,vel.x(),vel.y(),vel.z());
               dBodySetAngularVel(body,avel.x(),avel.y(),avel.z()); }
    return true;
  }
  return false;
}

/* ----------------------------------------------------------- Plane */
Plane::Plane(double a,double b,double c,double d):a(a),b(b),c(c),d(d){}
void Plane::init(const OdeHandle& h, double mass, char m){
  this->mode = m;
  if (!substanceManuallySet) substance = h.substance;
  if (m & Body){ body = dBodyCreate(h.world); setMass(mass, m & Density); }
  if (m & Geom){ geom = dCreatePlane(h.space, a,b,c,d); attachGeomAndSetColliderFlags(); }
}
void Plane::setMass(double mass, bool density){
  if (body){ dMass mm; dMassSetBox(&mm,mass,1000,1000,0.01); if(!density) dMassAdjust(&mm,mass); dBodySetMass(body,&mm); }
}

/* ----------------------------------------------------------- Box */
Box::Box(double lx,double ly,double lz):dim(lx,ly,lz){}
Box::Box(const Vec3& d):dim(d){}
void Box::init(const OdeHandle& h, double mass, char m){
  assert((m & Body) || (m & Geom));
  if (!substanceManuallySet) substance = h.substance;
  this->mode = m;
  if (m & Body){ body = dBodyCreate(h.world); setMass(mass, m & Density); }
  if (m & Geom){ geom = dCreateBox(h.space, dim.x(), dim.y(), dim.z()); attachGeomAndSetColliderFlags(); }
}
bool Box::getShapeMass(dMass& m, double mass, bool density) const {
  dMassSetBox(&m, mass, dim.x(), dim.y(), dim.z());
  if (!density) dMassAdjust(&m, mass);
  return true;
}
void Box::setMass(double mass, bool density){
  if (body){ dMass m; getShapeMass(m, mass, density); dBodySetMass(body, &m); }
}

/* ----------------------------------------------------------- Sphere */
Sphere::Sphere(double r):radius(r){}
void Sphere::init(const OdeHandle& h, double mass, char m){
  assert((m & Body) || (m & Geom));
  if (!substanceManuallySet) substance = h.substance;
  this->mode = m;
  if (m & Body){ body = dBodyCreate(h.world); setMass(mass, m & Density); }
  if (m & Geom){ geom = dCreateSphere(h.space, radius); attachGeomAndSetColliderFlags(); }
}
bool Sphere::getShapeMass(dMass& m, double mass, bool density) const {
  dMassSetSphere(&m, mass, radius);
  if (!density) dMassAdjust(&m, mass);
  return true;
}
void Sphere::setMass(double mass, bool density){
  if (body){ dMass m; getShapeMass(m, mass, density); dBodySetMass(body, &m); }
}

/* ----------------------------------------------------------- Capsule */
Capsule::Capsule(double r,double hgt):radius(r),height(hgt){}
void Capsule::init(const OdeHandle& h, double mass, char m){
  assert((m & Body) || (m & Geom));
  if (!substanceManuallySet) substance = h.substance;
  this->mode = m;
  if (m & Body){ body = dBodyCreate(h.world); setMass(mass, m & Density); }
  if (m & Geom){ geom = dCreateCapsule(h.space, radius, height); attachGeomAndSetColliderFlags(); }
}
bool Capsule::getShapeMass(dMass& m, double mass, bool density) const {
  dMassSetCapsule(&m, mass, 3, radius, height);   // direction 3 = local Z
  if (!density) dMassAdjust(&m, mass);
  return true;
}
void Capsule::setMass(double mass, bool density){
  if (body){ dMass m; getShapeMass(m, mass, density); dBodySetMass(body, &m); }
}

/* ----------------------------------------------------------- Cylinder */
Cylinder::Cylinder(double r,double hgt):radius(r),height(hgt){}
void Cylinder::init(const OdeHandle& h, double mass, char m){
  assert((m & Body) || (m & Geom));
  if (!substanceManuallySet) substance = h.substance;
  this->mode = m;
  if (m & Body){ body = dBodyCreate(h.world); setMass(mass, m & Density); }
  if (m & Geom){ geom = dCreateCylinder(h.space, radius, height); attachGeomAndSetColliderFlags(); }
}
bool Cylinder::getShapeMass(dMass& m, double mass, bool density) const {
  dMassSetCylinder(&m, mass, 3, radius, height);   // direction 3 = local Z
  if (!density) dMassAdjust(&m, mass);
  return true;
}
void Cylinder::setMass(double mass, bool density){
  if (body){ dMass m; getShapeMass(m, mass, density); dBodySetMass(body, &m); }
}

/* ----------------------------------------------------------- Ray */
Ray::Ray(double range):range(range),length(range){}
void Ray::init(const OdeHandle& h, double, char m){
  assert(!(m & Body) && (m & Geom));
  if (!substanceManuallySet) substance = h.substance;
  this->mode = m;
  geom = dCreateRay(h.space, range);
  dGeomRaySet(geom, 0,0,0, 0,0,1);   // origin at center, direction = local +Z (the documented convention)
  dGeomRaySetClosestHit(geom, 1);    // report the nearest hit (matters for trimesh/multi-hit targets)
  attachGeomAndSetColliderFlags();
}

/* ----------------------------------------------------------- Transform (geom transform) */
Transform::Transform(Primitive* parent, Primitive* child, const Pose& pose, bool deleteChild)
  : parent(parent), child(child), pose(pose), deleteChild(deleteChild) {
  assert(!dynamic_cast<Transform*>(child) && "nested Transform (a Transform as child) is unsupported");
}
Transform::~Transform(){ if (child && deleteChild) delete child; }

void Transform::init(const OdeHandle& h, double mass, char m){
  assert(parent && parent->getBody() && child);
  assert(child->getBody()==0 && child->getGeom()==0);   // child must be uninitialised
  this->mode = m | _Transform;
  if (!substanceManuallySet) substance = h.substance;

  geom = dCreateGeomTransform(h.space);
  dGeomTransformSetInfo(geom, 1);   // info=1 REQUIRED: a contact's geom.g1/g2 then resolves to the
                                    // TRANSFORM geom (whose body is the parent), so the collision
                                    // callback's dGeomGetBody attaches contacts to the parent body.
                                    // info=0 would expose the child (body 0) -> contacts wrongly
                                    // attach to the static world.
  dGeomTransformSetCleanup(geom, 0);

  OdeHandle childHandle(h);
  childHandle.space = 0;                                 // child geom inherits the transform's space
  child->init(childHandle, mass, (m & ~Body) | _Child);
  child->setPose(pose);                                  // offset in parent's local frame

  dGeomTransformSetGeom(geom, child->getGeom());
  dGeomSetBody(geom, parent->getBody());                 // ride the parent body
  dGeomSetData(geom, (void*)this);
  body = parent->getBody();                              // shared; not destroyed by us (_Transform)
  // An offset Transform geom adds COLLISION geometry but NOT mass/inertia to the parent
  // body (faithful to lpzrobots). Composing offset masses incrementally fights ODE's hard
  // "COM must be at the body origin" rule and corrupts under multiple transforms, so we keep
  // offset geoms massless. Correct robot inertia comes from one-shape-per-body + joints
  // (MENTAL_MODEL §5/§9). For a true composite, use getShapeMass() to build a combined,
  // COM-at-origin dMass yourself and set it on a single body.
  (void)mass;
}

void Transform::setMass(double mass, bool density){ child->setMass(mass, density); }

} // namespace mor
