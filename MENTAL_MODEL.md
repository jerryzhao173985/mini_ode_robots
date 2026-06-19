# The Mental Model: building robots on ODE the lpzrobots way

> This document is the *why*. It explains, grounded in the actual lpzrobots source and the
> ODE C/C++ usage skill (`.claude/skills/ode/`), how a physics-robotics engine is built on top
> of the **Open Dynamics Engine (ODE)**. The companion code in this folder (`mini_ode_robots/`)
> is the *what* — a dependency-free extraction of exactly these ideas.

---

## 0. The one idea ODE is built around

ODE keeps **dynamics** and **collision** as two separate worlds and bolts them together once per step.

```
   DYNAMICS world (dWorldID)              COLLISION world (dSpaceID)
   ┌───────────────────────┐             ┌───────────────────────┐
   │  dBody: mass + motion │             │  dGeom: shape only    │
   │  (position, velocity, │             │  (box/sphere/capsule, │
   │   force, torque)      │             │   used for dCollide)  │
   │  HAS NO SHAPE         │             │  HAS NO MASS          │
   └───────────┬───────────┘             └───────────┬───────────┘
               │      dGeomSetBody(geom, body)        │
               └──────────────  BRIDGE  ──────────────┘
```

- A **`dBody`** lives in a `dWorldID`. It has mass and integrates motion. It has **no shape**.
- A **`dGeom`** lives in a `dSpaceID`. It has a shape and is what collision detection sees. It has **no mass**.
- **`dGeomSetBody(geom, body)`** is the single most important call in the API — it ties a shape to
  a moving mass so they travel together. (ODE skill, hard rule #1.)
- A **joint** constrains two bodies (or one body to the static world, body `0`).
- A **contact joint** is a *throwaway* joint created at each collision point, every step, then discarded.

Everything lpzrobots does is a disciplined wrapper around this picture.

---

## 1. The canonical simulation loop (get this exactly right)

Every ODE program — and every lpzrobots simulation — is this loop. From the ODE skill `SKILL.md`
and verified in lpzrobots `simulation.cpp:1589` (`odeStep`):

```cpp
dInitODE2(0);                                  // 1. init, before ANY other ODE call
world = dWorldCreate();                         // 2. dynamics world
space = dHashSpaceCreate(0);                     // 3. broadphase collision space
contactgroup = dJointGroupCreate(0);             // 4. holds this step's contact joints
dWorldSetGravity(world, 0,0,-9.81);              //    default gravity is (0,0,0)!

// ... create bodies+geoms, link with dGeomSetBody, create joints ...

for (each step) {
    dSpaceCollide(space, data, &nearCallback);   //  a. broadphase → calls nearCallback per pair
    dWorldStep(world, dt);                        //  b. integrate one fixed timestep
    dJointGroupEmpty(contactgroup);               //  c. discard this step's contact joints
}
// teardown: dJointGroupDestroy, dSpaceDestroy, dWorldDestroy, dCloseODE
```

The three rules that bite everyone (ODE skill hard rules #2, #3, #4):
- **Init first** (`dInitODE2`); **empty the contact group every step** or contacts accumulate and
  corrupt the sim; **`dCollide` writes an ARRAY** — pass `dContact contact[N]` with stride `sizeof(dContact)`.

`dWorldStep` (accurate LCP, good for jointed chains) vs `dWorldQuickStep` (fast iterative, good for big
stacks) is a per-scene choice. lpzrobots defaults to `dWorldStep` (`simulation.cpp:1603`).

---

## 2. `OdeHandle` — the context bundle

ODE programs constantly need the world, the space, and the contact-joint group together. lpzrobots
bundles them so every constructor takes one handle instead of three globals (`utils/odehandle.h`):

```cpp
class OdeHandle {
  dWorldID world;          // dynamics
  dSpaceID space;          // collision (current space; can be nested)
  dJointGroupID jointGroup;// where contact joints are created each step
  Substance substance;     // default material for primitives made with this handle
  // + a set of "ignored geom pairs" and a list of nested spaces (collision filtering)
};
```

Two extra responsibilities live here and matter a lot for robots:

- **Ignored pairs.** When two primitives are joined (e.g. a wheel hinged to a chassis) you almost
  never want their geoms to also *collide*. `Joint::init` calls `odeHandle.addIgnoredPair(g1,g2)`
  and the collision callback skips them (`joint.cpp:57`, `simulation.cpp:1363`). This is how a robot
  doesn't fight itself.
- **Nested spaces.** A robot can get its own sub-space so its parts collide with the world but the
  broadphase stays cheap and self-collisions can be globally ignored (`createNewSimpleSpace`).

---

## 3. `Primitive` — a body+geom welded into one object

This is the heart of the extraction. A `Primitive` is the ODE duality of §0 turned into one C++
object whose `mode` bitfield says which halves it owns (`osg/primitive.h:89`):

```cpp
enum Modes { Body=1, Geom=2, Draw=4, Density=8, _Child=16, _Transform=32 };
//            │        │       │       │
//            │        │       │       └ interpret mass as density
//            │        │       └ has a visual (we DROP this — it's the only OSG coupling)
//            │        └ owns a dGeomID (collision)
//            └ owns a dBodyID (dynamics)
```

So:
- `Body|Geom` → a normal dynamic object (a falling box).
- `Geom` only → static scenery (the ground plane, a wall) — collidable but immovable.
- `Body` only → a point mass with no shape (rare).

`Primitive::init()` for every shape follows the same three-step recipe (`primitive.cpp`, e.g. Box at `:417`):

```cpp
if (mode & Body) { body = dBodyCreate(world); setMass(mass); }      // dynamics half
if (mode & Geom) { geom = dCreate<Shape>(space, dims...);           // collision half
                   attachGeomAndSetColliderFlags(); }                // the BRIDGE + filtering
if (mode & Draw) { /* OSG visual — DELETED in the extraction */ }
```

`attachGeomAndSetColliderFlags()` (`primitive.cpp:97`) is lpzrobots' wrapper around the bridge and is
worth reading closely, because it does three jobs at once:

```cpp
dGeomSetBody(geom, body);                  // 1. THE BRIDGE (only if there is a body)
dGeomSetCategoryBits(geom, Dyn|Stat...);   // 2. collision filtering: dynamic vs static
dGeomSetCollideBits(geom, ...);            //    (statics don't collide with statics)
dGeomSetData(geom, (void*)this);           // 3. back-pointer: geom → owning Primitive
```

Job #3 is the linchpin of the whole material system: when two geoms collide, the callback recovers
the two `Primitive*`s via `dGeomGetData`, and from each it reads a `Substance`. (See §6.)

### Shape → ODE call map (what each subclass actually does)

| `Primitive` | geom call | mass call | skill gotcha |
|---|---|---|---|
| `Box(x,y,z)` | `dCreateBox` | `dMassSetBox` | dims are **full** side lengths (hard rule #11) |
| `Sphere(r)` | `dCreateSphere` | `dMassSetSphere` | rolling spheres defeat auto-disable (rule #9) |
| `Capsule(r,h)` | `dCreateCapsule`¹ | `dMassSetCapsule(.,3,.)` | length **excludes** caps; dir `3`=local-Z |
| `Cylinder(r,h)` | `dCreateCylinder` | `dMassSetCylinder(.,3,.)` | dir `3`=local-Z |
| `Plane(a,b,c,d)` | `dCreatePlane` | (static, no body) | a geom with **no body** = static world |
| `Ray(len)` | `dCreateRay` | (no mass) | a sensing geom; reports nearest hit |
| `Transform(parent,child,pose)` | `dCreateGeomTransform` | child's mass | encapsulated geom — see §5 |

¹ lpzrobots' bundled `ode-dbl` calls it `dCreateCCylinder`; standard ODE 0.16 renamed it to
`dCreateCapsule` (ODE skill: "Things to never invent"). The extraction uses the modern name.

### Pose, mass, force — the body API a robot needs

```cpp
setPosition / setPose      → dBodySetPosition / dBodySetQuaternion (falls back to geom if no body)
getPosition / getPose      → dBodyGetPosition / dBodyGetRotation   (or geom)
getVel / getAngularVel     → dBodyGetLinearVel / dBodyGetAngularVel
applyForce / applyTorque   → dBodyAddForce / dBodyAddTorque  (world coordinates, reset each step)
setMass(m, cg, I…)         → dMassSetParameters + dBodySetMass
limitLinearVel/Angular     → a stability clamp on runaway velocities (a known ODE robustness hack)
```

---

## 4. The coordinate / rotation bridge (where silent bugs live)

ODE and the matrix math must agree on conventions or orientations break *silently* (the sim still
runs, things just point the wrong way). lpzrobots uses OpenSceneGraph's **row-vector** convention
(`v' = v · M`, translation in the **last row**). The extraction keeps that exact convention in a tiny
`Matrix` class so the ported math is identical.

Two conversions matter (`primitive.cpp:51-77`):

```cpp
// ODE stores rotation as dMatrix3 R (row-major 3×4) + position V (dReal[3]).
// Build a row-vector 4×4 = transpose(R) in the 3×3 block, V in the last row:
Pose osgPose(const double* V, const double* R) {
  return Pose(R[0],R[4],R[8], 0,    R[1],R[5],R[9], 0,
              R[2],R[6],R[10],0,    V[0],V[1],V[2],  1);
}
// Quaternion order differs: matrix lib uses (x,y,z,w); ODE uses (w,x,y,z) — "w-first" (rule #10).
dQuaternion q = { quat.w(), quat.x(), quat.y(), quat.z() };
```

Other invariants from the skill (§coordinate-frames, hard rule #10):
**angles are radians; quaternions are w-first `[w,x,y,z]`; ODE is right-handed, Z-up by convention.**

---

## 5. Composite bodies: `Transform` (encapsulated geoms)

A single rigid body often needs several shapes at offsets (a chassis = a box + a sensor mast, all one
rigid mass). ODE's tool for "a geom that is rigidly offset from its body" is the **geom transform**.
`Transform` (`primitive.cpp:677`) hides it:

```cpp
geom = dCreateGeomTransform(space);        // a wrapper geom
dGeomTransformSetInfo(geom, 1);
dGeomTransformSetCleanup(geom, 0);
child->init(.., space=0, .., _Child);       // child geom goes in NO space (inherited)
child->setPose(offsetPose);                 // offset in the parent body's local frame
dGeomTransformSetGeom(geom, child->getGeom());
dGeomSetBody(geom, parent->getBody());      // the whole thing rides the parent body
```

This is how you build one rigid body from several shapes without extra bodies or joints.

**Composite mass — and why an offset Transform geom is deliberately MASSLESS.** A tempting "fix"
is to fold the child's mass into the parent (parallel-axis). We tried it and an empirical audit
killed it: **ODE 0.16 hard-requires a body's centre of mass to sit at its origin** (`dBodySetMass`
aborts otherwise), so an off-centre child forces you to *recenter* — move the body reference to the
COM and re-offset every geom. Done **incrementally** (one `Transform::init` per part), that is
fragile: a *second* mass-bearing Transform recomputes the COM and double-moves the body, corrupting
the geometry — and it breaks lpzrobots' own two-bumper / four-wheel pattern (`nimm2.cpp`,
`fourwheeled.cpp`), where the offset geoms are bumpers that must stay massless. So the grounded,
faithful decision is lpzrobots': **an offset Transform geom adds collision only, never mass.** Correct
robot inertia comes from the §9 principle — *one shape per body, joined by joints* — which every
robot here follows. If you genuinely need a multi-shape rigid body, use the `getShapeMass()` utility
to assemble a single COM-at-origin `dMass` yourself and set it on one body. (Verified: offset geoms
are massless and `getShapeMass` returns correct per-shape inertia — `test_features` [7].)

---

## 6. `Substance` — intuitive materials instead of raw ERP/CFM

ODE's contact realism is controlled by `dSurfaceParameters`: `mu` (friction), and the
spring-damper pair **ERP** (error reduction — how hard to push interpenetration out) and **CFM**
(constraint force mixing — how soft the contact is). These are notoriously unintuitive. lpzrobots
replaces them with four physical scalars (`osg/substance.h:103`):

```cpp
class Substance { float roughness, slip, hardness, elasticity; CollisionCallback callback; };
```

and **derives** the ODE surface from the two colliding materials with real physics
(`substance.cpp:54`):

```
mu       = roughness1 · roughness2                                   // friction multiplies
kp       = 100 · (h1·h2)/(h1+h2)                                     // two springs in SERIES
kd       = 50  · ((1-e1)·h2 + (1-e2)·h1)/(h1+h2)                     // damping from elasticity
soft_erp = h·kp / (h·kp + kd)                                        // standard ODE mapping…
soft_cfm = 1    / (h·kp + kd)                                        // …(h = timestep)
slip1=slip2 = slip1 + slip2
mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 [| dContactSlip1|2]
```

The `soft_erp`/`soft_cfm` formulas are *exactly* the spring-damper→ERP/CFM identity from the ODE
manual and the skill's `foundations/erp-cfm-friction.md`. The `100×`/`50×` are empirical scalings so
the friendly `hardness` range (≈5–200) lands in good ERP/CFM territory.

Presets ship as factory methods: `Plastic`(default), `Metal`, `Rubber`, `Foam`, `Snow`, `NoContact`,
and an **anisotropic friction** mode (different friction along an axis — for snake scales) implemented
via the per-substance `callback`. The callback hook returns `0`=ignore collision, `1`=use default
params, `2`=params already set — which is how `NoContact` and anisotropic friction plug in
(`substance.cpp:202,222`).

---

## 7. `Joint` — constraints with motors

A `Joint` connects two `Primitive`s. Every subclass is a thin wrapper: create → attach → set
anchor/axis (`joint.cpp`). The hierarchy mirrors the joint's degrees of freedom:

```
Joint                       (0 axes: Fixed, Ball)
 └ OneAxisJoint             (1 axis : Hinge, Slider)
    └ TwoAxisJoint          (2 axes : Hinge2, Universal)
```

```cpp
// HingeJoint (joint.cpp:195) — the prototype every robot leg/wheel uses:
joint = dJointCreateHinge(world, 0);
dJointAttach(joint, part1->body, part2->body);   // part2's BODY may be 0 (static world)
dJointSetHingeAnchor(joint, ax,ay,az);            // where it pivots
dJointSetHingeAxis(joint, dx,dy,dz);              // what it rotates about
// control & sensing:
addForce1(t)       → dJointAddHingeTorque         // actuate
getPosition1()     → dJointGetHingeAngle          // sense angle  (radians)
getPosition1Rate() → dJointGetHingeAngleRate      // sense angular velocity
setParam(p,v)      → dJointSetHingeParam          // dParamLoStop/HiStop/Vel/FMax …
```

Two robot-critical details:
- **`Joint::init` ignores the pair** of connected geoms (§2) so the joined parts don't collide.
- **Motors** are done the ODE way: set `dParamVel` (target speed) + `dParamFMax` (max force) to make
  a *servo*, or `addForce` to apply raw torque. Limit stops via `dParamLoStop`/`HiStop` are set
  **after** the axis (setting the axis re-zeros the angle — skill hard rule #12).
- **Feedback**: `dJointSetFeedback` lets you read the constraint force/torque (used for torque
  sensors).

---

## 8. The collision callback in full (the per-step glue)

`Simulation::nearCallback` (`simulation.cpp:1348`) is the canonical near-callback specialized with
lpzrobots' two features — nested spaces and substances:

```cpp
void nearCallback(void* data, dGeomID o1, dGeomID o2) {
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {       // 1. a space, not a geom →
      dSpaceCollide2(o1, o2, data, &nearCallback);  //    recurse (handles nested robot spaces)
      return;
  }
  if (odeHandle.isIgnoredPair(o1,o2)) return;        // 2. skip joint-connected parts
  Primitive* p1 = (Primitive*)dGeomGetData(o1);      // 3. recover owners (set in §3 job #3)
  Primitive* p2 = (Primitive*)dGeomGetData(o2);
  dContact contact[80];                              // 4. an ARRAY (skill hard rule #4)
  int n = dCollide(o1,o2,80,&contact[0].geom,sizeof(dContact));
  if (n>0) {
     // 5. let each substance's callback have a say, else derive default surface:
     Substance::getSurfaceParams(surf, p1->substance, p2->substance, dt);   // §6
     for (i in 0..n) {                                // 6. one contact joint per point
        contact[i].surface = surf;
        dJointID c = dJointCreateContact(world, jointGroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
     }
  }
}
```

This is precisely the skill's canonical near-callback, plus: space recursion, ignored-pair filtering,
and material-driven surface parameters. The extraction reproduces it verbatim (minus the OSG
contact-debug drawing).

---

## 9. `OdeRobot` — a robot is a bag of primitives + joints + I/O

`OdeRobot` (`robots/oderobot.h:64`) is deliberately simple. A robot *is*:

```cpp
std::vector<Primitive*> objects;   // its rigid parts
std::vector<Joint*>     joints;    // how they connect
// + a controller-facing interface:
int  getSensorNumber();  int getMotorNumber();
int  getSensors(double* s, int n);     // read joint angles/rates, body state → s[]
void setMotors(const double* m, int n);// write m[] → joint motor targets/torques
void place(const Pose&);               // build the parts at a pose (placeIntern)
```

A concrete robot (e.g. the two-wheeled **Nimm2**) just:
1. in `placeIntern(pose)`: creates a body `Primitive` + wheel `Primitive`s, sets masses, creates
   `HingeJoint`s for the wheels, sets motor params, registers ignored pairs (done by joint init),
   stores them in `objects`/`joints`;
2. in `setMotors`: maps the controller's motor command to `joint->setParam(dParamVel, …)`;
3. in `getSensors`: reads `joint->getPosition1Rate()` (wheel speed) back to the controller.

The controller itself (the learning algorithms in `selforg/`) is **out of scope** for this extraction
— we keep only the physics-robotics substrate. A robot here is driven by a plain function or a hand
written policy.

---

## 9b. The control & sensing layer (motors and sensors)

A robot needs to *act* and *perceive*. lpzrobots layers two reusable elements on top of joints
and geoms; the extraction keeps both, pure-ODE:

- **`OneAxisServo`** (`motors/oneaxisservo`) — *position control* for a hinge/slider. It runs a
  **PID** on the joint angle and emits a torque via `joint->addForce1()`, with the joint's
  limit-stops (`dParamLoStop/HiStop`) set from the travel bounds and a velocity clamp for
  stability. `set(pos∈[-1,1])` commands a scaled target; the servo must be re-driven every step
  (the PID acts each call). This is what the **Arm** uses to hold a pose against gravity.
- **`RaySensor`** (`sensors/raysensor`) — *distance/IR sensing*. A `Ray` geom is rigidly mounted
  on the body via a `Transform` and points along its local +Z. It piggybacks on the collision
  pipeline: a per-`Substance` **callback** records each hit's `contact.geom.depth` and returns
  `0` so **no contact joint is made** — the ray is a pure probe. `sense()` latches the nearest hit
  per step. This single class exercises three engine features at once (Ray + Transform + the
  Substance callback), and it's why those exist.

The control loop a robot sees each step is therefore: `sense()` (read joints/rays) → controller →
`doInternalStuff()` (drive servos / motors) → `Simulation::step()` (collide + integrate).

## 10. What we keep vs. drop in the extraction

| lpzrobots dependency | Decision | Replacement |
|---|---|---|
| **ODE** (`ode-dbl`) | **KEEP** | standard ODE 0.16 (`ode-config`), modern API names |
| OpenSceneGraph (`osg::Vec3/Vec4/Matrix`, `OSGPrimitive`, color, texture) | **DROP** | a ~300-line header `osg_compat.h`: `Vec3/Vec4/Matrix/Quat` (same row-vector convention) |
| `selforg` (`Storeable`, `AbstractRobot`, `matrix::Matrix`, `Position`, `HashSet`) | **DROP** | `std::unordered_set`, plain virtual robot base, our own math |
| Qt configurator, guilogger, video, sound | **DROP** | not physics |
| `QMP_*` threading macros, tasked simulation | **DROP** | single-threaded (skill default) |
| Substance / contact model | **KEEP** (verbatim physics) | identical formulas |
| Primitive / Joint / OdeHandle / collision loop | **KEEP** (re-expressed) | identical ODE calls, no visuals |

The result is a self-contained C++ engine whose only external dependency is `libode`, and which
preserves lpzrobots' *physics* exactly while shedding everything else.

---

## 11. Verifying it's correct (not just that it ran)

Per the skill's `foundations/verifying-simulations.md`, "it compiled and didn't crash" is not
verification. The extraction ships a **headless self-check** that asserts physical invariants:

- **Energy**: a dropped, settling body's total mechanical energy must *dissipate*, never grow
  (energy growth ⇒ solver instability).
- **Rest**: objects on the ground come to rest (auto-disable / damping working).
- **No NaN**: positions/velocities stay finite (no explosion).
- **Joints stay satisfied**: a hinge pendulum's bodies stay at the constrained distance.
- **Trajectory sanity**: the driven robot actually translates when both wheels spin forward, and
  turns when they differ.

These checks are the real definition of "the physics engine works."
