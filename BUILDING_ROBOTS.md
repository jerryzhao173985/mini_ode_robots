# Building a new robot on mini_ode_robots

This is the practical "how to extend" guide. Read `MENTAL_MODEL.md` first for the *why*;
this shows the *how*, grounded in the real classes. A robot here is just:

> **a bag of `Primitive`s (rigid parts) + `Joint`s (how they connect) + an I/O interface
> (`getSensors`/`setMotors`) the controller talks to.**

You subclass `OdeRobot`, build the parts in `placeIntern(pose)`, and expose sensors/motors —
either by overriding the `*Intern` hooks or by **attaching** `Sensor`/`Motor` objects.

---

## 1. The construction recipe (`placeIntern`)

Everything is built relative to a `pose` (where the robot is placed). The three idioms:

```cpp
void MyRobot::placeIntern(const Pose& pose) {
  // (a) a PART: create -> init(mass) -> place at  localPose * pose
  Box* body = new Box(0.4, 0.3, 0.1);
  body->init(odeHandle, 1.0);                 // dynamic body + collision geom, 1 kg
  body->setPose(pose);                        // body sits at the robot origin
  dBodySetAutoDisableFlag(body->getBody(), 0);// robots are ACTIVE — never auto-sleep (gotcha!)
  objects.push_back(body);                    // register for cleanup + tracking

  // (b) a part offset in the robot's local frame: localPose * pose
  Capsule* leg = new Capsule(0.05, 0.4);
  leg->init(odeHandle, 0.2);
  Pose local = Pose::rotate(Vec3(0,0,1), Vec3(1,0,0))   // capsule long axis Z -> X
             * Pose::translate(0.3, 0, -0.2);
  leg->setPose(local * pose);
  dBodySetAutoDisableFlag(leg->getBody(), 0);
  objects.push_back(leg);

  // (c) a JOINT: anchor is a local POINT * pose; axis is a local DIRECTION * pose
  Pos  anchor = Vec3(0.3, 0, 0) * pose;       // a point   (w=1): rotated AND translated
  Axis axis   = Axis(0, 1, 0) * pose;         // a direction(w=0): rotated only
  HingeJoint* hj = new HingeJoint(leg, body, anchor, axis);
  hj->init(odeHandle);                        // also marks leg<->body an ignored collision pair
  joints.push_back(hj);
}
```

Why `Vec3*pose` vs `Axis*pose`: a `Pos` is homogeneous-w=1 (a point — translates), an `Axis`
is w=0 (a direction — rotation only). This is what makes a robot place correctly at any pose,
including rotated ones (verified for Nimm4 under yaw).

---

## 2. Actuation — use the velocity servo

For position control prefer `OneAxisServoVel` (stable at any gain; `OneAxisServo` is a compliant
force PID — see `servo.h`). Construct it on a `OneAxisJoint` and drive it every step:

```cpp
// in the class:  std::vector<OneAxisServoVel> servos;
servos.emplace_back(hj, /*min*/-1.4, /*max*/1.4, /*power(FMax)*/30.0, /*damp*/0.3, /*maxVel*/12.0);
// each physics step:
void MyRobot::doInternalStuffIntern(GlobalData&){ for (auto& s : servos) s.set(target[i]); }
```

Hinge motors can also be driven raw: `hinge->setParam(dParamVel, v); hinge->setParam(dParamFMax, f);`
(this is what Nimm4's wheels do).

---

## 3. The controller I/O — two ways

**Option A — override the `*Intern` hooks** (best when the robot has fixed channels):

```cpp
int  getSensorNumberIntern() override { return nJoints; }
int  getMotorNumberIntern()  override { return nJoints; }
int  getSensorsIntern(double* s, int n) override { /* write joint angles */ return k; }
void setMotorsIntern(const double* m, int n) override { /* store targets */ }
void doInternalStuffIntern(GlobalData&) override { /* drive servos */ }
```

**Option B — attach `Sensor`/`Motor` objects** (best for composable, optional sensing). The robot
*owns* them; `getSensors`/`setMotors` append their channels automatically:

```cpp
RaySensor* eye = new RaySensor(5.0);
eye->init(odeHandle, getMainPrimitive(), Pose::rotate(Vec3(0,0,1), Vec3(1,0,0)));
addSensor(eye);                                   // appends 1 distance channel
addSensor(new JointSensor(hj, /*withRate=*/true));// appends [angle, rate]
addMotor (new OneAxisServoVel(hj, -1,1, 20));     // appends 1 command channel
```

Both compose: a robot can have `*Intern` channels *and* attached sensors/motors;
`getSensorNumber()` is the sum. See `examples/demo_sensors.cpp`.

---

## 4. Running and recording

```cpp
Simulation sim; sim.init();
Plane ground; ground.init(sim.getOdeHandle(), 0);
MyRobot robot(sim.getOdeHandle()); robot.place(Pos(0,0, restHeight));
sim.addRobot(&robot);

Logger log("run.csv", {"x","y"});
sim.run(2000, [&](Simulation& s, double t){
  double cmd[N] = {...}; robot.setMotors(cmd, N);     // your controller
  log.log(t, { robot.getPosition().x(), robot.getPosition().y() });
});
// NB: declare `sim` BEFORE robots so it is destroyed LAST (it owns the ODE world).
```

`Simulation::run` calls, per step: each robot's `sense()` (attached sensors + `senseIntern`),
then your control callback, then `doInternalStuff()` (attached motors + `doInternalStuffIntern`),
then the physics step.

---

## 5. Gotchas checklist (the engine guards/handles these — don't re-introduce them)

- **Auto-disable**: call `dBodySetAutoDisableFlag(body, 0)` on every robot body, or a resting,
  motor-commanded body stays frozen (a sleeping body ignores joint motors).
- **Inertia**: one shape per body. Offset `Transform` geoms are collision-only (massless). For a
  true multi-shape body, assemble a COM-at-origin `dMass` via `getShapeMass()` and set it once.
- **Universal/Hinge2 axes** must be linearly independent (a guard asserts this — parallel axes
  abort ODE otherwise).
- **Lifetime**: `Simulation` must outlive all `Primitive`/`Joint`s (declare it first). The engine
  guards wrong-order teardown, but correct order avoids leaks.
- **Self-collision**: joint-connected parts are auto-ignored. For broader self-collision control,
  build the robot in its own sub-space: `odeHandle.createNewSimpleSpace(parent, /*ignoreInside*/true)`.
- **Tracking**: `getMainPrimitive()` defaults to `objects[0]`; override it if part 0 is a static base.

---

## 6. A complete minimal robot

A 1-DOF powered pendulum (`pendulum.*` is left as an exercise; this is the whole pattern):

```cpp
class Pendulum : public OdeRobot {
public:
  Pendulum(const OdeHandle& h) : OdeRobot(h, "Pendulum") {}
  int  getMotorNumberIntern()  override { return 1; }
  int  getSensorNumberIntern() override { return 1; }
  int  getSensorsIntern(double* s, int n) override { if(n>0){ s[0]=servo->get(); return 1;} return 0; }
  void setMotorsIntern(const double* m, int n) override { if(n>0) target = m[0]; }
  void doInternalStuffIntern(GlobalData&) override { servo->set(target); }
  void placeIntern(const Pose& pose) override {
    Box* base = new Box(0.1,0.1,0.1); base->init(odeHandle,0,Primitive::Geom); base->setPose(pose);
    objects.push_back(base);                                   // static pivot (geom only)
    Capsule* arm = new Capsule(0.03,0.5); arm->init(odeHandle,0.5);
    arm->setPose(Pose::rotate(Vec3(0,0,1),Vec3(1,0,0)) * Pose::translate(0.25,0,0) * pose);
    dBodySetAutoDisableFlag(arm->getBody(),0); objects.push_back(arm);
    HingeJoint* hj = new HingeJoint(arm, base, Vec3(0,0,0)*pose, Axis(0,1,0)*pose);
    hj->init(odeHandle);
    servo = new OneAxisServoVel(hj, -3.14, 3.14, 20); joints.push_back(hj);
  }
  ~Pendulum() override { /* servo owned here; OdeRobot::cleanup frees joints/primitives */ delete servo; }
private:
  OneAxisServoVel* servo = nullptr; double target = 0;
};
```

That's the entire surface area: parts, one joint, one servo, four I/O hooks. Copy `robots/nimm4.*`
(wheeled), `robots/arm.*` (servo chain), or `robots/snake.*` (anisotropic friction) as starting points.
