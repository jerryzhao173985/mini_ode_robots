# mini_ode_robots — a dependency-free ODE physics-robotics engine

A clean, self-contained C++ extraction of the **physics-robotics core** of
[lpzrobots](https://github.com/georgmartius/lpzrobots), built directly on the
**Open Dynamics Engine (ODE)**. It keeps lpzrobots' way of *building robots on ODE*
— the body+geom `Primitive`, the `Joint` hierarchy, the material-based contact
model, the collision/step loop, the robot base class — and **drops everything else**
(OpenSceneGraph rendering, the `selforg` controllers/matrix lib, Qt, threading).

**Its only external dependency is `libode`.** Built and verified against ODE 0.16.6.

> Read **[`MENTAL_MODEL.md`](MENTAL_MODEL.md)** first — it explains the *why*:
> how dynamics/collision are bolted together, and how each class maps to ODE,
> grounded in both the lpzrobots source and the ODE usage skill.
> For a one-page tour of the development journey, decisions, and what is proven, see
> **[`DESIGN_NOTES.md`](DESIGN_NOTES.md)**; to add your own robot, **[`BUILDING_ROBOTS.md`](BUILDING_ROBOTS.md)**.

---

## Build & run

Requires a C++17 compiler and ODE (`brew install ode`; `ode-config` on PATH).

```sh
make            # builds build/libmor.a + the six example binaries
make test       # builds and runs the headless verification suite
```

Or with CMake (portable; finds ODE via pkg-config or ode-config):

```sh
cmake -S . -B build && cmake --build build && ctest --test-dir build
```

Six headless, self-checking programs (no window, no GPU):

| binary | what it proves |
|---|---|
| `build/test_math` | the math layer's conventions round-trip **exactly through ODE** |
| `build/selfcheck` | physical invariants: energy dissipates, hinge constraints hold, stacks are stable, no NaN |
| `build/test_features` | Transform composite, Ray sensor distance, anisotropic friction, Slider/Universal/Ball constraints |
| `build/demo_robot` | the `Nimm4` robot drives straight and turns (trajectory sanity) |
| `build/demo_arm` | the servo `Arm` tracks position commands (tip moves, joint sign follows command) |
| `build/demo_snake` | the `Snake` crawls forward from undulation + anisotropic friction (≈7 m); logs a CSV |
| `build/demo_sensors` | composable I/O: a stock `Nimm4` with an attached ray + joint sensor (`addSensor`) |

All exit `0` on success and print per-check `PASS`/`FAIL`. The assertions use tolerant thresholds
(e.g. "snake travels > 0.1 m", "low-friction slide > 1.5× the high-friction one") so they hold
across ODE builds; the *typical* measurements on a current build are: ray reads 1.000/2.000 m;
anisotropic box slides ≈8.5 m vs ≈0.07 m (≈127×); snake travels ≈7.5 m; arm tip moves ≈1.9 m.

---

## What's inside

```
include/mor/          the engine (≈ the lpzrobots ode_robots core, OSG/selforg removed)
  osg_compat.h        Vec3/Vec4/Quat/Matrix — replaces OpenSceneGraph (row-vector convention)
  pose_types.h        Pos / Axis / Pose + the ODE<->matrix bridge (poseFromOde / odeRotation)
  globaldata.h        OdeConfig — the tunable physics "tokens" (timestep, gravity, ERP/CFM)
  odehandle.h         world + space + contact-group bundle, ignored pairs, nested spaces, clock
  substance.h         the material model (roughness/slip/hardness/elasticity -> ODE surface)
  primitive.h         Box/Sphere/Capsule/Cylinder/Plane/Ray/Transform (body+geom)
  joint.h             Fixed/Hinge/Hinge2/Universal/Ball/Slider (+ motors, feedback)
  pid.h / servo.h     PID + OneAxisServo(Vel): position control for a hinge/slider joint
  sensor.h / motor.h  attachable Sensor/Motor interfaces (composable robot I/O)
  raysensor.h         distance/IR sensor (Ray + Transform + Substance callback) — a Sensor
  jointsensor.h       proprioceptive Sensor: joint angle (+rate)
  obstacles.h         Playground (static walls) + passive box/sphere helpers
  logger.h            minimal header-only CSV logger for headless experiments
  simulation.h        ODE lifecycle + collision near-callback + the canonical step loop
  oderobot.h          robot base: primitives+joints + aggregating getSensors/setMotors + addSensor/addMotor
src/                  the implementations
robots/               nimm4 (differential drive), arm (servo chain), snake (anisotropic friction)
examples/             test_math, selfcheck, test_features, demo_robot, demo_arm, demo_snake, demo_sensors
```

**To build your own robot, read [`BUILDING_ROBOTS.md`](BUILDING_ROBOTS.md)** — the practical
extension guide (construction recipe, servos, composable sensors/motors, gotchas, a minimal robot).

### Source → lpzrobots provenance

| this engine | extracted from | changes |
|---|---|---|
| `osg_compat.h` | `osg::Vec3/Vec4/Matrix/Quat` (OpenSceneGraph) | reimplemented; same row-vector convention |
| `pose_types.h` | `utils/pos.h`, `pose.h`, `axis.h`, `osg/primitive.cpp` (osgPose) | OSG bases removed |
| `odehandle.{h,cpp}` | `utils/odehandle.{h,cpp}` | `selforg` HashSet → `std::unordered_set`; ODE lifecycle moved to `Simulation` |
| `substance.{h,cpp}` | `osg/substance.{h,cpp}` | **physics verbatim**; only the OSG include dropped |
| `primitive.{h,cpp}` | `osg/primitive.{h,cpp}` | `Draw`/OSG/`Storeable` removed; `dCreateCCylinder`→`dCreateCapsule` |
| `joint.{h,cpp}` | `osg/joint.{h,cpp}` | visuals removed; same `dJointCreate*` calls |
| `simulation.{h,cpp}` | `simulation.cpp` (`odeStep`, `nearCallback`) | GUI/threading/rendering removed |
| `oderobot.{h,cpp}` | `robots/oderobot.{h,cpp}` | `AbstractRobot`/`Storeable`/Sensor-Motor system simplified |
| `pid.h` / `servo.h` | `motors/pid.*`, `motors/oneaxisservo.*` | PID + OneAxisServo (force/torque variant), self-contained |
| `raysensor.{h,cpp}` | `sensors/raysensor.*` | distance sensor; visuals removed |
| `obstacles.h` | `obstacles/playground.h`, `passivebox.h`, `passivesphere.h` | minimal static walls + passive bodies |
| `robots/nimm4.*` | `robots/nimm2.*` (in spirit) | clean 4-wheel differential drive |
| `robots/arm.*` | `robots/arm*.*` (in spirit) | servo-controlled hinge chain |
| `robots/snake.*` | `robots/schlange*.*` (in spirit) | anisotropic-friction undulating snake |

---

## A minimal program

```cpp
#include "mor/simulation.h"
#include "mor/primitive.h"
using namespace mor;

int main() {
  Simulation sim; sim.init();             // dInitODE2 + world/space/contactgroup
  OdeHandle& oh = sim.getOdeHandle();

  Plane ground;  ground.init(oh, 0);      // static ground (a geom, no body)
  Sphere ball(0.3); ball.init(oh, 1.0);   // dynamic body + geom, mass 1 kg
  ball.setPosition(Pos(0,0,5));
  ball.setSubstance(Substance::getRubber(15));

  for (int i=0;i<300;i++) sim.step();     // collide -> integrate -> empty contacts
  ball.getPosition().print("rest: ");     // ~ (0,0,0.3): resting on the ground
}                                          // RAII: ball, ground destroyed before sim closes ODE
```

## Hard rules honoured (from the ODE skill)

- link with the **`c++` driver + `ode-config`**, define **no precision macro**, assert
  `dCheckConfiguration("ODE_double_precision")` at startup;
- `dInitODE2` first; **empty the contact group every step**; `dCollide` into a **`dContact` array**;
- `dGeomSetBody` bridges every body↔geom; box dims are **full** lengths; capsule axis is **local Z**;
- quaternions are **w-first**; angles are **radians**;
- **Simulation must outlive all Primitives/Joints** (it owns the ODE lifecycle);
- verified **headless** (energy/constraints/no-NaN), not just "it compiled."
