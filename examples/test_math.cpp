/* test_math.cpp — validate the math layer's conventions AGAINST ODE itself.
 *
 * If the row-vector convention or the (x,y,z,w)<->(w,x,y,z) quaternion reorder
 * is wrong, orientations break silently. So we close the loop through ODE:
 *   ODE body pose  --poseFromOde-->  Pose  --get(quat)/odeRotation-->  ODE
 * and assert we get back what we put in, plus that point transforms agree.
 */
#include "mor/pose_types.h"
#include <ode/ode.h>
#include <cstdio>
#include <cmath>

using namespace mor;
static int failures = 0;
static void check(bool ok, const char* what){
  std::printf("  [%s] %s\n", ok?"PASS":"FAIL", what);
  if(!ok) failures++;
}
static bool close(double a, double b, double eps=1e-9){ return std::fabs(a-b)<eps; }
static bool vclose(const Vec3&a,const Vec3&b,double eps=1e-7){
  return close(a.x(),b.x(),eps)&&close(a.y(),b.y(),eps)&&close(a.z(),b.z(),eps);
}

int main(){
  dInitODE2(0);
  check(dCheckConfiguration("ODE_double_precision"), "ODE is double precision");

  dWorldID world = dWorldCreate();
  dBodyID b = dBodyCreate(world);

  // 1) set a known orientation on the ODE body, round-trip through our Pose
  dQuaternion qin = {0.0,0.0,0.0,0.0};
  { // 37 deg about a tilted axis
    double ang=0.6457718; double ax=0.3,ay=-0.8,az=0.5; double n=std::sqrt(ax*ax+ay*ay+az*az);
    ax/=n;ay/=n;az/=n; double s=std::sin(ang/2);
    qin[0]=std::cos(ang/2); qin[1]=ax*s; qin[2]=ay*s; qin[3]=az*s; // ODE order w,x,y,z
  }
  dBodySetPosition(b, 1.0, 2.0, 3.0);
  dBodySetQuaternion(b, qin);

  // read ODE rotation, build our Pose
  Pose p = poseFromOde(dBodyGetPosition(b), dBodyGetRotation(b));

  // translation must match
  check(vclose(p.getTrans(), Vec3(1,2,3)), "Pose.getTrans matches ODE position");

  // extract quaternion from our Pose, push it back to ODE the lpzrobots way
  Quat q; p.get(q);
  dReal quat[4] = { q.w(), q.x(), q.y(), q.z() }; // (x,y,z,w)->(w,x,y,z)
  dBodyID b2 = dBodyCreate(world);
  dBodySetQuaternion(b2, quat);

  const dReal* R1 = dBodyGetRotation(b);
  const dReal* R2 = dBodyGetRotation(b2);
  bool Rmatch=true; for(int i=0;i<12;i++) if(!close(R1[i],R2[i],1e-7)) Rmatch=false;
  check(Rmatch, "quaternion round-trip Pose->ODE reproduces rotation matrix");

  // 2) point transform agreement: a local point through our Pose == ODE's mapping
  Vec3 local(0.5, -0.2, 0.1);
  Vec3 viaPose = local * p;                       // our row-vector transform
  dVector3 odeWorld;
  dBodyGetRelPointPos(b, local.x(), local.y(), local.z(), odeWorld);
  check(vclose(viaPose, Vec3(odeWorld[0],odeWorld[1],odeWorld[2])),
        "point transform v*Pose matches dBodyGetRelPointPos");

  // 3) inverse: world point back to local
  Vec3 backLocal = viaPose * Pose::inverse(p);
  check(vclose(backLocal, local, 1e-6), "inverse Pose maps world point back to local");

  // 4) rotate(from,to) actually aligns the vectors
  Vec3 from(0,0,1), to(1,0,0);
  Pose r = Pose::rotate(from, to);
  check(vclose(from*r, to, 1e-9), "rotate(from,to) maps from onto to");

  dBodyDestroy(b); dBodyDestroy(b2); dWorldDestroy(world);  // teardown before dCloseODE (skill rule)
  dCloseODE();
  std::printf("\nMATH TEST: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
