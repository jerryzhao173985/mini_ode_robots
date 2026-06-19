/* mini_ode_robots — pose_types.h
 *
 * The thin lpzrobots-style aliases over the math layer:
 *   Pos   : a point/position   (lpzrobots: class Pos : osg::Vec3)
 *   Axis  : a direction (w=0)   (lpzrobots: class Axis: osg::Vec4)
 *   Pose  : a 4x4 rigid pose    (lpzrobots: typedef osg::Matrix Pose)
 *
 * Plus the ODE <-> Pose bridge (lpzrobots: osg/primitive.cpp osgPose / odeRotation).
 */
#ifndef MOR_POSE_TYPES_H
#define MOR_POSE_TYPES_H

#include "osg_compat.h"
#include <ode/ode.h>

namespace mor {

/// A position. Adds the handful of operators lpzrobots' Pos provided.
class Pos : public Vec3 {
public:
  Pos() : Vec3() {}
  Pos(double x, double y, double z) : Vec3(x,y,z) {}
  Pos(const Vec3& v) : Vec3(v) {}
  Pos(const Vec4& v) : Vec3(v.x(),v.y(),v.z()) {}
  explicit Pos(const dReal* v) : Vec3(v) {}
  /// componentwise product (lpzrobots Pos::operator&)
  Pos operator&(const Pos& p) const { return Pos(x()*p.x(), y()*p.y(), z()*p.z()); }
};

/// A 3D axis, stored homogeneous with w=0 (a direction, not a point).
class Axis : public Vec4 {
public:
  Axis() : Vec4() {}
  Axis(double x, double y, double z) : Vec4(x,y,z,0) {}
  Axis(const Vec4& v) : Vec4(v) { w()=0; }
  Axis(const Vec3& v) : Vec4(v,0) {}
  explicit Axis(const dReal* v) : Vec4(v[0],v[1],v[2],0) {}
  Vec3 vec3() const { return Vec3(x(),y(),z()); }
  Axis crossProduct(const Axis& a) const {
    return Axis(y()*a.z()-z()*a.y(), z()*a.x()-x()*a.z(), x()*a.y()-y()*a.x());
  }
};

/// A rigid pose is just a 4x4 matrix (row-vector convention).
typedef Matrix Pose;

/* ----- the ODE <-> Pose bridge (verbatim semantics from lpzrobots) ----- */

/// Build a Pose from an ODE position vector V[3] and rotation matrix R[12].
/// ODE's R is row-major 3x4; we place its transpose in the 3x3 block and the
/// position in the last row — exactly lpzrobots' osgPose().
inline Pose poseFromOde(const dReal* V, const dReal* R){
  return Pose(R[0], R[4], R[8],  0,
              R[1], R[5], R[9],  0,
              R[2], R[6], R[10], 0,
              V[0], V[1], V[2],  1);
}
Pose poseOfBody(dBodyID body);
Pose poseOfGeom(dGeomID geom);

/// Convert the rotation part of a Pose into an ODE rotation matrix.
/// Quaternion is reordered (x,y,z,w) -> ODE's (w,x,y,z): the "w-first" rule.
inline void odeRotationFromPose(const Pose& pose, dMatrix3& odematrix){
  Quat q; pose.get(q);
  dQuaternion quat = { q.w(), q.x(), q.y(), q.z() };
  dQtoR(quat, odematrix);
}

} // namespace mor
#endif
