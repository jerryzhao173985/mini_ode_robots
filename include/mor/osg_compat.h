/* mini_ode_robots — osg_compat.h
 *
 * A tiny, dependency-free replacement for the handful of OpenSceneGraph math
 * types that lpzrobots' physics core leans on: Vec3, Vec4, Quat, Matrix.
 *
 * The ONE thing that must be preserved exactly is OSG's convention, because
 * lpzrobots' ODE<->matrix bridge (osgPose) was written against it:
 *
 *   - ROW-VECTOR convention:  v' = v * M     (point is a row on the LEFT)
 *   - 4x4 row-major storage,  translation lives in the LAST ROW (m[3][0..2])
 *   - M = A * B  applies A then B   (v*(A*B) = (v*A)*B)
 *   - Quaternion stored (x,y,z,w);  ODE uses (w,x,y,z) — reordered at the bridge.
 *
 * Everything is double precision to match double-precision libode.
 * Grounded in: ODE skill foundations/coordinate-frames.md (radians, w-first quats).
 */
#ifndef MOR_OSG_COMPAT_H
#define MOR_OSG_COMPAT_H

#include <cmath>
#include <cstdio>
#include <utility>
#include <ode/ode.h>

namespace mor {

/* ------------------------------------------------------------------ Vec3 */
class Vec3 {
public:
  double _v[3];
  Vec3() { _v[0]=_v[1]=_v[2]=0; }
  Vec3(double x, double y, double z) { _v[0]=x; _v[1]=y; _v[2]=z; }
  explicit Vec3(const dReal* v) { _v[0]=v[0]; _v[1]=v[1]; _v[2]=v[2]; }

  double  x() const { return _v[0]; }  double& x() { return _v[0]; }
  double  y() const { return _v[1]; }  double& y() { return _v[1]; }
  double  z() const { return _v[2]; }  double& z() { return _v[2]; }
  double  operator[](int i) const { return _v[i]; }
  double& operator[](int i)       { return _v[i]; }
  const double* ptr() const { return _v; }

  Vec3 operator+(const Vec3& o) const { return Vec3(_v[0]+o._v[0], _v[1]+o._v[1], _v[2]+o._v[2]); }
  Vec3 operator-(const Vec3& o) const { return Vec3(_v[0]-o._v[0], _v[1]-o._v[1], _v[2]-o._v[2]); }
  Vec3 operator-()             const { return Vec3(-_v[0], -_v[1], -_v[2]); }
  Vec3 operator*(double s)     const { return Vec3(_v[0]*s, _v[1]*s, _v[2]*s); }
  /// dot product (OSG semantics: Vec3 * Vec3 == dot)
  double operator*(const Vec3& o) const { return _v[0]*o._v[0]+_v[1]*o._v[1]+_v[2]*o._v[2]; }
  /// cross product (OSG semantics: Vec3 ^ Vec3 == cross)
  Vec3 operator^(const Vec3& o) const {
    return Vec3(_v[1]*o._v[2]-_v[2]*o._v[1],
                _v[2]*o._v[0]-_v[0]*o._v[2],
                _v[0]*o._v[1]-_v[1]*o._v[0]);
  }
  Vec3& operator+=(const Vec3& o){ _v[0]+=o._v[0]; _v[1]+=o._v[1]; _v[2]+=o._v[2]; return *this; }

  double length2() const { return _v[0]*_v[0]+_v[1]*_v[1]+_v[2]*_v[2]; }
  double length()  const { return std::sqrt(length2()); }
  double normalize() {
    double n = length();
    if (n > 0) { double inv = 1.0/n; _v[0]*=inv; _v[1]*=inv; _v[2]*=inv; }
    return n;
  }
  bool isNaN() const { return std::isnan(_v[0])||std::isnan(_v[1])||std::isnan(_v[2]); }
  void print(const char* tag="") const { std::printf("%s(%g, %g, %g)\n", tag, _v[0],_v[1],_v[2]); }
};
inline Vec3 operator*(double s, const Vec3& v) { return v*s; }

/* ------------------------------------------------------------------ Vec4 */
class Vec4 {
public:
  double _v[4];
  Vec4() { _v[0]=_v[1]=_v[2]=_v[3]=0; }
  Vec4(double x, double y, double z, double w) { _v[0]=x; _v[1]=y; _v[2]=z; _v[3]=w; }
  Vec4(const Vec3& v, double w) { _v[0]=v.x(); _v[1]=v.y(); _v[2]=v.z(); _v[3]=w; }

  double  x() const { return _v[0]; }  double& x() { return _v[0]; }
  double  y() const { return _v[1]; }  double& y() { return _v[1]; }
  double  z() const { return _v[2]; }  double& z() { return _v[2]; }
  double  w() const { return _v[3]; }  double& w() { return _v[3]; }
  Vec3 vec3() const { return Vec3(_v[0],_v[1],_v[2]); }
};

/* ------------------------------------------------------------------ Quat
 * Unit quaternion (x,y,z,w). Matches OSG's component order. */
class Quat {
public:
  double _v[4]; // x,y,z,w
  Quat() { _v[0]=_v[1]=_v[2]=0; _v[3]=1; }
  Quat(double x,double y,double z,double w){ _v[0]=x;_v[1]=y;_v[2]=z;_v[3]=w; }
  double x() const { return _v[0]; }
  double y() const { return _v[1]; }
  double z() const { return _v[2]; }
  double w() const { return _v[3]; }
};

/* ---------------------------------------------------------------- Matrix
 * 4x4, row-major, row-vector convention (v' = v*M). */
class Matrix {
public:
  double _m[4][4];

  Matrix() { makeIdentity(); }
  /// 16 values in ROW-MAJOR order (a00,a01,a02,a03, a10,...). This matches the
  /// argument order lpzrobots' osgPose(V,R) uses to build a pose.
  Matrix(double a00,double a01,double a02,double a03,
         double a10,double a11,double a12,double a13,
         double a20,double a21,double a22,double a23,
         double a30,double a31,double a32,double a33) {
    _m[0][0]=a00;_m[0][1]=a01;_m[0][2]=a02;_m[0][3]=a03;
    _m[1][0]=a10;_m[1][1]=a11;_m[1][2]=a12;_m[1][3]=a13;
    _m[2][0]=a20;_m[2][1]=a21;_m[2][2]=a22;_m[2][3]=a23;
    _m[3][0]=a30;_m[3][1]=a31;_m[3][2]=a32;_m[3][3]=a33;
  }

  void makeIdentity() {
    for (int i=0;i<4;i++) for (int j=0;j<4;j++) _m[i][j]=(i==j)?1.0:0.0;
  }
  const double* ptr() const { return &_m[0][0]; }
  double*       ptr()       { return &_m[0][0]; }

  /// translation lives in the last row (row-vector convention)
  Vec3 getTrans() const { return Vec3(_m[3][0], _m[3][1], _m[3][2]); }
  void setTrans(const Vec3& t){ _m[3][0]=t.x(); _m[3][1]=t.y(); _m[3][2]=t.z(); }

  /* --- factories (static), mirroring osg::Matrix::translate / ::rotate --- */
  static Matrix translate(double x,double y,double z){
    Matrix m; m._m[3][0]=x; m._m[3][1]=y; m._m[3][2]=z; return m;
  }
  static Matrix translate(const Vec3& v){ return translate(v.x(),v.y(),v.z()); }

  static Matrix rotate(const Quat& q){ Matrix m; m.makeRotate(q); return m; }
  static Matrix rotate(double angleRad, const Vec3& axis){
    Matrix m; m.makeRotate(angleRad, axis); return m;
  }
  /// rotation taking unit vector `from` onto unit vector `to`
  static Matrix rotate(const Vec3& from, const Vec3& to){
    Matrix m; m.makeRotate(from, to); return m;
  }

  static Matrix inverse(const Matrix& m){ Matrix r; r.invert(m); return r; }

  /* --- quaternion <-> matrix (row-vector form: this is OSG's Quat::get) --- */
  void makeRotate(const Quat& q){
    double x=q.x(),y=q.y(),z=q.z(),w=q.w();
    double n = x*x+y*y+z*z+w*w;
    double s = (n>0.0)? 2.0/n : 0.0;
    double xs=x*s, ys=y*s, zs=z*s;
    double wx=w*xs, wy=w*ys, wz=w*zs;
    double xx=x*xs, xy=x*ys, xz=x*zs;
    double yy=y*ys, yz=y*zs, zz=z*zs;
    makeIdentity();
    _m[0][0]=1.0-(yy+zz); _m[0][1]=xy+wz;       _m[0][2]=xz-wy;
    _m[1][0]=xy-wz;       _m[1][1]=1.0-(xx+zz); _m[1][2]=yz+wx;
    _m[2][0]=xz+wy;       _m[2][1]=yz-wx;       _m[2][2]=1.0-(xx+yy);
  }
  void makeRotate(double angle, const Vec3& axisIn){
    Vec3 axis = axisIn; double len = axis.normalize();
    if (len < 1e-12) { makeIdentity(); return; }
    double s = std::sin(angle*0.5), c = std::cos(angle*0.5);
    makeRotate(Quat(axis.x()*s, axis.y()*s, axis.z()*s, c));
  }
  void makeRotate(const Vec3& fromIn, const Vec3& toIn){
    Vec3 from=fromIn, to=toIn; from.normalize(); to.normalize();
    double d = from*to;
    if (d >= 1.0-1e-12){ makeIdentity(); return; }          // already aligned
    if (d <= -1.0+1e-12){                                    // opposite: 180deg about any perp axis
      Vec3 axis = Vec3(1,0,0)^from;
      if (axis.length2() < 1e-9) axis = Vec3(0,1,0)^from;
      axis.normalize();
      makeRotate(M_PI, axis);
      return;
    }
    Vec3 axis = from^to; double angle = std::acos(d);
    makeRotate(angle, axis);
  }

  /// extract a quaternion (x,y,z,w) from the rotation part (row-vector form)
  void get(Quat& q) const {
    double t = _m[0][0]+_m[1][1]+_m[2][2];
    if (t > 0.0){
      double s = std::sqrt(t+1.0)*2.0;          // s=4w
      q = Quat((_m[1][2]-_m[2][1])/s,
               (_m[2][0]-_m[0][2])/s,
               (_m[0][1]-_m[1][0])/s,
               0.25*s);
    } else if (_m[0][0] > _m[1][1] && _m[0][0] > _m[2][2]){
      double s = std::sqrt(1.0+_m[0][0]-_m[1][1]-_m[2][2])*2.0; // s=4x
      q = Quat(0.25*s,
               (_m[0][1]+_m[1][0])/s,
               (_m[2][0]+_m[0][2])/s,
               (_m[1][2]-_m[2][1])/s);
    } else if (_m[1][1] > _m[2][2]){
      double s = std::sqrt(1.0+_m[1][1]-_m[0][0]-_m[2][2])*2.0; // s=4y
      q = Quat((_m[0][1]+_m[1][0])/s,
               0.25*s,
               (_m[1][2]+_m[2][1])/s,
               (_m[2][0]-_m[0][2])/s);
    } else {
      double s = std::sqrt(1.0+_m[2][2]-_m[0][0]-_m[1][1])*2.0; // s=4z
      q = Quat((_m[2][0]+_m[0][2])/s,
               (_m[1][2]+_m[2][1])/s,
               0.25*s,
               (_m[0][1]-_m[1][0])/s);
    }
  }

  /* --- matrix * matrix (standard row-major product) --- */
  Matrix operator*(const Matrix& rhs) const {
    Matrix r;
    for (int i=0;i<4;i++)
      for (int j=0;j<4;j++){
        double s=0; for (int k=0;k<4;k++) s += _m[i][k]*rhs._m[k][j];
        r._m[i][j]=s;
      }
    return r;
  }

  /* --- v * M  (postMult: translation taken from the last ROW) --- */
  Vec3 postMult(const Vec3& v) const {
    double d = 1.0/(_m[0][3]*v.x()+_m[1][3]*v.y()+_m[2][3]*v.z()+_m[3][3]);
    return Vec3((_m[0][0]*v.x()+_m[1][0]*v.y()+_m[2][0]*v.z()+_m[3][0])*d,
                (_m[0][1]*v.x()+_m[1][1]*v.y()+_m[2][1]*v.z()+_m[3][1])*d,
                (_m[0][2]*v.x()+_m[1][2]*v.y()+_m[2][2]*v.z()+_m[3][2])*d);
  }
  Vec4 postMult(const Vec4& v) const {
    return Vec4(_m[0][0]*v.x()+_m[1][0]*v.y()+_m[2][0]*v.z()+_m[3][0]*v.w(),
                _m[0][1]*v.x()+_m[1][1]*v.y()+_m[2][1]*v.z()+_m[3][1]*v.w(),
                _m[0][2]*v.x()+_m[1][2]*v.y()+_m[2][2]*v.z()+_m[3][2]*v.w(),
                _m[0][3]*v.x()+_m[1][3]*v.y()+_m[2][3]*v.z()+_m[3][3]*v.w());
  }

  /// general 4x4 inverse (Gauss-Jordan). Robust enough for affine poses.
  void invert(const Matrix& in){
    double a[4][8];
    for (int i=0;i<4;i++){ for (int j=0;j<4;j++){ a[i][j]=in._m[i][j]; a[i][j+4]=(i==j)?1.0:0.0; } }
    for (int col=0; col<4; col++){
      int piv=col; double best=std::fabs(a[col][col]);
      for (int r=col+1;r<4;r++){ if (std::fabs(a[r][col])>best){ best=std::fabs(a[r][col]); piv=r; } }
      if (piv!=col) for (int j=0;j<8;j++) std::swap(a[col][j],a[piv][j]);
      double d=a[col][col]; if (std::fabs(d)<1e-18) d=1e-18;
      for (int j=0;j<8;j++) a[col][j]/=d;
      for (int r=0;r<4;r++) if (r!=col){ double f=a[r][col]; for (int j=0;j<8;j++) a[r][j]-=f*a[col][j]; }
    }
    for (int i=0;i<4;i++) for (int j=0;j<4;j++) _m[i][j]=a[i][j+4];
  }
};

/* v * M  (the everyday transform call sites in the ported code) */
inline Vec3 operator*(const Vec3& v, const Matrix& m){ return m.postMult(v); }
inline Vec4 operator*(const Vec4& v, const Matrix& m){ return m.postMult(v); }

} // namespace mor
#endif
