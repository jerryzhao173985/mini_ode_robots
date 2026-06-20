/* test_features.cpp — headless verification of the features no other test covered:
 *   1. Transform (composite/offset geom rides the parent body)
 *   2. RaySensor (measures a known distance)
 *   3. Anisotropic friction (slides farther along the low-friction axis)
 *   4. Slider / Universal / Ball joint constraints hold
 */
#include "mor/simulation.h"
#include "mor/primitive.h"
#include "mor/joint.h"
#include "mor/raysensor.h"
#include "mor/servo.h"
#include "mor/bodysensors.h"
#include "mor/jointsensor.h"
#include "mor/contactsensor.h"
#include "mor/angularmotor.h"
#include <cstdio>
#include <cmath>

using namespace mor;
static int failures = 0;
static void check(bool ok, const char* what){
  std::printf("  [%s] %s\n", ok?"PASS":"FAIL", what); if(!ok) failures++;
}
static bool fin(const Vec3& v){ return std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z()); }

/* 1) Transform: a body whose only collision shape is an OFFSET child geom must
      rest at a height set by that offset — proving the geom-transform rides the body. */
static void test_transform(){
  std::printf("[1] Transform composite geom\n");
  Simulation sim; sim.init();
  OdeHandle& oh = sim.getOdeHandle();
  Plane ground; ground.init(oh, 0);

  Box parent(0.2,0.2,0.2);                 // body + its own geom
  parent.init(oh, 1.0);
  parent.setPosition(Pos(0,0,2.0));
  Sphere* foot = new Sphere(0.2);          // child geom, offset 0.5 below the body
  Transform foize(&parent, foot, Pose::translate(0,0,-0.5));
  foize.init(oh, 1.0);

  bool nan=false; for(int i=0;i<600;i++){ sim.step(); if(!fin(parent.getPosition())) {nan=true;break;} }
  double z = parent.getPosition().z();
  check(!nan, "no NaN with composite body");
  // lowest contact = parent box bottom (z-0.1) vs offset sphere bottom (z-0.5-0.2=z-0.7).
  // the sphere is lower, so the body rests at z ~= 0.7.
  check(std::fabs(z - 0.7) < 0.06, "composite rests at height set by the OFFSET child geom (z~0.7)");
}

/* 2) RaySensor: a downward ray from a body at height H over the ground reads ~H. */
static void test_ray(){
  std::printf("[2] RaySensor distance\n");
  Simulation sim; sim.init();
  OdeHandle& oh = sim.getOdeHandle();
  Plane ground; ground.init(oh, 0);

  // body-only owner (no geom) so the ray cannot hit its own shape; pinned in place
  Box owner(0.1,0.1,0.1); owner.init(oh, 1.0, Primitive::Body);
  owner.setPosition(Pos(0,0,1.0));
  dBodySetKinematic(owner.getBody());

  RaySensor ray(5.0);
  ray.init(oh, &owner, Pose::rotate(Vec3(0,0,1), Vec3(0,0,-1)));  // point the ray's +Z downward

  sim.step(); ray.sense(sim.getGlobalData());
  double d1 = ray.get();
  owner.setPosition(Pos(0,0,2.0));
  sim.step(); ray.sense(sim.getGlobalData());
  double d2 = ray.get();

  std::printf("    measured: at z=1 -> %.3f ; at z=2 -> %.3f\n", d1, d2);
  check(std::fabs(d1 - 1.0) < 0.05, "ray reads ~1.0 m to ground from height 1");
  check(std::fabs(d2 - 2.0) < 0.05, "ray reads ~2.0 m to ground from height 2");
}

/* 3) Anisotropic friction: identical boxes pushed along the LOW-friction axis vs
      the HIGH-friction axis — the low-friction one must travel noticeably farther. */
static void test_anisotropic(){
  std::printf("[3] Anisotropic friction directionality\n");
  Simulation sim; sim.init();
  OdeHandle& oh = sim.getOdeHandle();
  Plane ground; ground.init(oh, 0);

  auto makeBox = [&](double vx, double vy){
    Box* b = new Box(0.3,0.3,0.3);
    Substance s = Substance::getRubber(20);
    s.toAnisotropFriction(0.15, Axis(1,0,0));   // low friction along world X
    b->setSubstance(s);
    b->init(oh, 1.0);
    b->setPosition(Pos(0,0,0.151));
    dBodySetLinearVel(b->getBody(), vx, vy, 0);
    return b;
  };
  Box* along = makeBox(2.0, 0.0);   // pushed along X (low friction)
  Box* across= makeBox(0.0, 2.0);   // pushed along Y (high friction)

  bool nan=false;
  for(int i=0;i<300;i++){ sim.step(); if(!fin(along->getPosition())||!fin(across->getPosition())){nan=true;break;} }
  double dAlong  = std::fabs(along->getPosition().x());
  double dAcross = std::fabs(across->getPosition().y());
  std::printf("    travelled: along low-friction X = %.3f ; across high-friction Y = %.3f\n", dAlong, dAcross);
  check(!nan, "no NaN with anisotropic friction");
  check(dAlong > 1.5 * dAcross, "slides farther along the low-friction axis (anisotropy works)");
  delete along; delete across;
}

/* 4) Slider / Universal / Ball joints: constraints stay satisfied. */
static void test_joints(){
  std::printf("[4] Slider / Universal / Ball constraints\n");

  { // SLIDER: parts stay laterally aligned while sliding along Z
    Simulation sim; sim.init(); OdeHandle& oh = sim.getOdeHandle();
    Box base(0.2,0.2,0.2); base.init(oh,1.0,Primitive::Body); base.setPosition(Pos(0,0,2));
    dBodySetKinematic(base.getBody());
    Box slid(0.2,0.2,0.2); slid.init(oh,1.0); slid.setPosition(Pos(0,0,1.5));
    SliderJoint sj(&slid,&base, Pos(0,0,1.75), Axis(0,0,1)); sj.init(oh);
    double maxLat=0; bool nan=false;
    for(int i=0;i<300;i++){ sim.step(); Pos p=slid.getPosition(); if(!fin(p)){nan=true;break;}
      maxLat=std::max(maxLat, std::sqrt(p.x()*p.x()+p.y()*p.y())); }
    check(!nan && maxLat < 1e-2, "slider keeps parts laterally aligned (only slides along axis)");
  }
  auto pendulumHolds = [&](const char* name, int which){
    Simulation sim; sim.init(); sim.setGravity(-9.81); OdeHandle& oh = sim.getOdeHandle();
    Pos pivot(0,0,2);
    Box anchor(0.1,0.1,0.1); anchor.init(oh,1.0,Primitive::Body); anchor.setPosition(pivot);
    dBodySetKinematic(anchor.getBody());
    Sphere bob(0.1); bob.init(oh,1.0); bob.setPosition(pivot + Vec3(0.8,0,0));
    Joint* j;
    if (which==0) j = new UniversalJoint(&anchor,&bob,pivot,Axis(0,1,0),Axis(1,0,0));
    else          j = new BallJoint(&anchor,&bob,pivot);
    j->init(oh);
    double L0=(bob.getPosition()-Pos(pivot)).length(), maxErr=0, reach=0; bool nan=false;
    for(int i=0;i<400;i++){ sim.step(); Pos p=bob.getPosition(); if(!fin(p)){nan=true;break;}
      maxErr=std::max(maxErr,std::fabs((p-Pos(pivot)).length()-L0)); reach=std::max(reach,p.x()); }
    delete j;
    check(!nan && maxErr < 5e-3 && reach > 0.7, name);
  };
  pendulumHolds("universal joint holds bob at fixed radius and swings", 0);
  pendulumHolds("ball joint holds bob at fixed radius and swings", 1);
}

/* 5) New introspection/config APIs: Vec3 gravity and per-step contact count. */
static void test_new_apis(){
  std::printf("[5] setGravity(Vec3) + getLastStepNumContacts\n");
  { // sideways gravity accelerates a free body along +X
    Simulation sim; sim.init();
    sim.setGravity(Vec3(4.0, 0, -9.81));        // tilt the world
    Sphere ball(0.2); ball.init(sim.getOdeHandle(), 1.0);
    ball.setPosition(Pos(0,0,5));               // no ground -> free fall
    for(int i=0;i<50;i++) sim.step();
    double vx = ball.getVel().x();
    std::printf("    vx under sideways gravity = %.3f (expect ~+2.0 after 0.5s)\n", vx);
    check(vx > 1.0, "Vec3 gravity accelerates a free body sideways");
  }
  { // a body resting on the ground generates contact points
    Simulation sim; sim.init();
    Plane g; g.init(sim.getOdeHandle(), 0);
    Box b(0.4,0.4,0.4); b.init(sim.getOdeHandle(), 1.0); b.setPosition(Pos(0,0,0.2));
    for(int i=0;i<100;i++) sim.step();          // settle
    long nc = sim.getLastStepNumContacts();
    std::printf("    resting-box contacts last step = %ld\n", nc);
    check(nc > 0, "getLastStepNumContacts reports ground contacts");
  }
}

/* 6) Velocity servo (OneAxisServoVel) is a CORRECT position controller: it must
      settle to the commanded angle with ~zero steady-state error and NO limit
      cycle, even at very high gain (where a force PID would oscillate). */
static void test_servo_convergence(){
  std::printf("[6] velocity servo convergence (no limit cycle, no SSE)\n");
  for (double power : {30.0, 200.0}){          // include a deliberately huge gain
    Simulation sim; sim.init(); sim.setGravity(0);
    Box base(0.1,0.1,0.1); base.init(sim.getOdeHandle(),1,Primitive::Body);
    base.setPosition(Pos(0,0,1)); dBodySetKinematic(base.getBody());
    Sphere bob(0.1); bob.init(sim.getOdeHandle(),1); bob.setPosition(Pos(0.5,0,1));
    HingeJoint hj(&bob,&base,Pos(0,0,1),Axis(0,1,0)); hj.init(sim.getOdeHandle());
    OneAxisServoVel servo(&hj, -1.5, 1.5, power, 0.3, 12.0);

    const double cmd = 0.5;                    // centered -> 0.75 rad target
    const double targetAngle = (cmd+1)*(1.5-(-1.5))/2 + (-1.5);
    double tail[800]; int t=0;
    for (int i=0;i<3000;i++){ servo.set(cmd); sim.step();
      if (i>=2200) tail[t++] = hj.getPosition1(); }
    double mn=1e9,mx=-1e9; for(int i=0;i<t;i++){ mn=std::min(mn,tail[i]); mx=std::max(mx,tail[i]); }
    double achieved = tail[t-1], osc = mx-mn;
    std::printf("    power=%5.0f: achieved=%.4f target=%.4f err=%+.3f deg  tailOsc=%.5f rad\n",
                power, achieved, targetAngle, (achieved-targetAngle)*180/M_PI, osc);
    char m1[96], m2[96];
    std::snprintf(m1,sizeof m1,"settles to commanded angle (power=%.0f, |err|<1deg)", power);
    std::snprintf(m2,sizeof m2,"no limit cycle (power=%.0f, tail osc < 0.01 rad)", power);
    check(std::fabs(achieved-targetAngle) < 0.018, m1);   // <~1 deg steady-state error
    check(osc < 0.01, m2);
  }
}

/* 7) Transform policy: an offset Transform geom is COLLISION-ONLY (massless), so the
      parent body's mass = its primary shape and its origin does not shift. (Correct
      composite inertia comes from one-shape-per-body + joints; for a true composite,
      build a COM-at-origin dMass yourself via the getShapeMass utility, verified here.) */
static void test_transform_mass_policy(){
  std::printf("[7] Transform massless-offset policy + getShapeMass utility\n");
  Simulation sim; sim.init(); sim.setGravity(0);
  Box* parent = new Box(0.2,0.2,0.2); parent->init(sim.getOdeHandle(), 1.0);
  parent->setPosition(Pos(0,0,0));
  Sphere* off = new Sphere(0.2);
  Transform t(parent, off, Pose::translate(1.0,0,0));
  t.init(sim.getOdeHandle(), 10.0);                 // mass arg ignored for an offset geom
  dMass m; dBodyGetMass(parent->getBody(), &m);
  Vec3 boxg(dGeomGetPosition(parent->getGeom()));
  std::printf("    body mass=%.3f (exp 1)  primary geom=(%.3f,%.3f,%.3f) (exp 0,0,0)\n",
              m.mass, boxg.x(),boxg.y(),boxg.z());
  check(std::fabs(m.mass-1.0) < 1e-9, "offset Transform geom is MASSLESS (body mass = primary shape)");
  check(boxg.length() < 1e-9,         "primary geom stays at body origin (no COM shift / no multi-transform corruption)");
  // getShapeMass utility (for manual composite construction) returns correct inertia:
  Sphere s(0.3); dMass sm; bool ok = s.getShapeMass(sm, 2.0, false);
  double Iexp = (2.0/5.0)*2.0*0.09;   // 2/5 m r^2
  check(ok && std::fabs(sm.mass-2.0)<1e-9 && std::fabs(sm.I[0]-Iexp)<1e-6,
        "getShapeMass utility returns correct per-shape mass + inertia");
}

/* 8) Sub-space collision: createNewSimpleSpace makes the verified-correct nested-space
      plumbing usable. ignore_inside=false collides internal pairs exactly ONCE (no double
      count from the dSpaceCollide top + per-subspace loop); ignore_inside=true suppresses. */
static void test_subspaces(){
  std::printf("[8] sub-space collision (createNewSimpleSpace)\n");
  auto contactsInSub = [](bool ignoreInside){
    Simulation sim; sim.init(); sim.setGravity(0);
    OdeHandle sub(sim.getOdeHandle());
    sub.createNewSimpleSpace(sim.getOdeHandle().space, ignoreInside);
    Sphere* a = new Sphere(0.3); a->init(sub,1); a->setPosition(Pos(0,0,0));
    Sphere* b = new Sphere(0.3); b->init(sub,1); b->setPosition(Pos(0.4,0,0)); // overlap
    sim.step();
    long n = sim.getLastStepNumContacts();
    delete a; delete b;
    return n;
  };
  long collided = contactsInSub(false);
  long ignored  = contactsInSub(true);
  std::printf("    ignore_inside=false -> %ld contacts (exp 1, no double-count) ; =true -> %ld (exp 0)\n",
              collided, ignored);
  check(collided == 1, "registered sub-space collides internal pair exactly once (no double-count)");
  check(ignored  == 0, "ignore_inside=true sub-space suppresses internal collisions");
}

/* 9) Proprioceptive sensors (Speed / AxisOrientation / ForceTorque / Contact). */
static void test_proprioception(){
  std::printf("[9] proprioceptive sensors\n");
  { // AxisOrientationSensor: up-axis is (0,0,1) upright, tilts to (1,0,0) at 90deg about Y
    Simulation sim; sim.init(); sim.setGravity(0);
    Box b(0.2,0.2,0.2); b.init(sim.getOdeHandle(),1); b.setPosition(Pos(0,0,1));
    AxisOrientationSensor up(&b);
    double s[3]; up.get(s,3);
    check(std::fabs(s[2]-1.0) < 1e-6, "AxisOrientationSensor up-axis = (0,0,1) when upright");
    b.setPose(Pose::rotate(M_PI/2, Vec3(0,1,0)) * Pose::translate(0,0,1));
    up.get(s,3);
    check(std::fabs(s[0]-1.0) < 1e-3, "up-axis tilts to ~(1,0,0) when rotated 90deg about Y");
  }
  { // SpeedSensor reads body velocity
    Simulation sim; sim.init(); sim.setGravity(0);
    Sphere b(0.2); b.init(sim.getOdeHandle(),1); b.setPosition(Pos(0,0,1));
    dBodySetLinearVel(b.getBody(), 1.5, 0, 0);
    SpeedSensor sp(&b); double s[3]; sp.get(s,3);
    check(std::fabs(s[0]-1.5) < 1e-6, "SpeedSensor reads body linear velocity");
  }
  { // ForceTorqueSensor: a hinge holding a mass under gravity carries torque
    Simulation sim; sim.init(); sim.setGravity(-9.81);
    Box anchor(0.1,0.1,0.1); anchor.init(sim.getOdeHandle(),1,Primitive::Body);
    anchor.setPosition(Pos(0,0,2)); dBodySetKinematic(anchor.getBody());
    Sphere bob(0.2); bob.init(sim.getOdeHandle(),1); bob.setPosition(Pos(0.6,0,2));
    HingeJoint hj(&bob,&anchor,Pos(0,0,2),Axis(0,1,0)); hj.init(sim.getOdeHandle());
    ForceTorqueSensor ft(&hj);
    for(int i=0;i<10;i++) sim.step();
    double s[3]; ft.get(s,3);
    double tmag = std::sqrt(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]);
    check(tmag > 1e-3, "ForceTorqueSensor reports nonzero joint torque under load");
  }
  { // ContactSensor: 1 when resting on ground, 0 in the air
    Simulation sim; sim.init();
    Plane g; g.init(sim.getOdeHandle(),0);
    Box rest(0.4,0.4,0.4); rest.init(sim.getOdeHandle(),1); rest.setPosition(Pos(0,0,0.2));
    Box air(0.2,0.2,0.2);  air.init(sim.getOdeHandle(),1);  air.setPosition(Pos(6,6,6));
    ContactSensor cs(&rest), ca(&air);
    for(int i=0;i<80;i++){ sim.step(); cs.sense(sim.getGlobalData()); ca.sense(sim.getGlobalData()); }
    double a[1], r[1]; cs.get(r,1); ca.get(a,1);
    check(r[0] > 0.5, "ContactSensor = 1 when the box rests on the ground");
    check(a[0] < 0.5, "ContactSensor = 0 for a body in the air");
  }
}

/* 10) AngularMotor: actively drive a free body about an axis, and a BALL joint (3-DOF). */
static void test_angularmotor(){
  std::printf("[10] AngularMotor (active multi-axis drive)\n");
  { // free body driven about global Z to a commanded angular velocity
    Simulation sim; sim.init(); sim.setGravity(0);
    Box b(0.3,0.3,0.3); b.init(sim.getOdeHandle(),1); b.setPosition(Pos(0,0,1));
    AngularMotor am(sim.getOdeHandle(), &b, nullptr, { Axis(0,0,1) }, /*power*/5.0, /*maxVel*/8.0, /*global*/0);
    double cmd[1]={1.0}; am.set(cmd,1);
    for(int i=0;i<150;i++){ am.act(sim.getGlobalData()); sim.step(); }
    Pos w = b.getAngularVel();
    std::printf("    free body wz=%.3f (target 8.0), wx=%.3f wy=%.3f\n", w.z(), w.x(), w.y());
    check(std::fabs(w.z()-8.0) < 1.0, "drives a free body to commanded angular velocity about Z");
    check(std::fabs(w.x())<0.5 && std::fabs(w.y())<0.5, "no spurious rotation on the other axes");
  }
  { // a BALL joint actively rotated about Y — impossible with the joint's own (absent) motor
    Simulation sim; sim.init(); sim.setGravity(0);
    Box anchor(0.1,0.1,0.1); anchor.init(sim.getOdeHandle(),1,Primitive::Body);
    anchor.setPosition(Pos(2,0,1)); dBodySetKinematic(anchor.getBody());
    Sphere bob(0.15); bob.init(sim.getOdeHandle(),1); bob.setPosition(Pos(2,0,1));
    BallJoint bj(&bob,&anchor,Pos(2,0,1)); bj.init(sim.getOdeHandle());
    AngularMotor am3(sim.getOdeHandle(), &bob, &anchor,
                     { Axis(1,0,0), Axis(0,1,0), Axis(0,0,1) }, 5.0, 6.0, /*global*/0);
    double c3[3]={0,1.0,0}; am3.set(c3,3);
    for(int i=0;i<150;i++){ am3.act(sim.getGlobalData()); sim.step(); }
    Pos w = bob.getAngularVel();
    std::printf("    ball-joint wy=%.3f (driven), wx=%.3f wz=%.3f (held)\n", w.y(), w.x(), w.z());
    check(std::fabs(w.y()) > 2.0, "AngularMotor actively drives a BALL joint about a chosen axis (3-DOF)");
    check(std::fabs(w.x())<1.0 && std::fabs(w.z())<1.0, "the other two ball-joint axes are held near zero");
  }
}

/* 11) Contact stability: soft-contact materials + maxCorrectingVel keep a resting box
      jitter-free (the surface layer is intentionally off by default — see globaldata.h). */
static void test_contact_stability(){
  std::printf("[11] contact stability (soft contacts + maxCorrectingVel)\n");
  Simulation sim; sim.init();
  Plane g; g.init(sim.getOdeHandle(),0);
  Box box(0.5,0.5,0.5); box.init(sim.getOdeHandle(),5.0); box.setPosition(Pos(0,0,0.25));
  for(int i=0;i<200;i++) sim.step();                 // settle
  double mn=1e9, mx=-1e9;
  for(int i=0;i<100;i++){ sim.step(); double z=box.getPosition().z(); mn=std::min(mn,z); mx=std::max(mx,z); }
  std::printf("    resting z jitter = %.2e m\n", mx-mn);
  check(mx-mn < 1e-3, "resting box has negligible vertical jitter (stable contact)");
}

/* 12) TwoAxisServoVel: independent position control of BOTH universal-joint axes. */
static void test_twoaxis_servo(){
  std::printf("[12] TwoAxisServoVel (2-axis position control)\n");
  Simulation sim; sim.init(); sim.setGravity(0);
  Box base(0.1,0.1,0.1); base.init(sim.getOdeHandle(),1,Primitive::Body);
  base.setPosition(Pos(0,0,1)); dBodySetKinematic(base.getBody());
  Sphere bob(0.1); bob.init(sim.getOdeHandle(),1); bob.setPosition(Pos(0.5,0,1));
  UniversalJoint uj(&bob,&base,Pos(0,0,1),Axis(1,0,0),Axis(0,0,1)); uj.init(sim.getOdeHandle());
  TwoAxisServoVel servo(&uj, -1.0,1.0, -1.0,1.0, /*power*/60, /*maxVel*/12);
  for(int i=0;i<2500;i++){ servo.set(0.5,-0.5); sim.step(); }   // set() applies both axes
  double a1=uj.getPosition1(), a2=uj.getPosition2();
  std::printf("    axis1=%.3f (target 0.5) axis2=%.3f (target -0.5)\n", a1, a2);
  check(std::fabs(a1-0.5)  < 0.03, "TwoAxisServoVel settles axis1 to commanded angle");
  check(std::fabs(a2+0.5)  < 0.03, "TwoAxisServoVel settles axis2 to commanded angle");
}

int main(){
  test_transform();
  test_ray();
  test_anisotropic();
  test_joints();
  test_new_apis();
  test_servo_convergence();
  test_transform_mass_policy();
  test_subspaces();
  test_proprioception();
  test_angularmotor();
  test_contact_stability();
  test_twoaxis_servo();
  std::printf("\nFEATURE TESTS: %s\n", failures==0 ? "ALL PASS" : "FAILURES PRESENT");
  return failures==0 ? 0 : 1;
}
