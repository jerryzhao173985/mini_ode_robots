# Design notes — the development journey, distilled

A concise record of *what* was built, *why* the key decisions were made, and *what is proven*.
For the full mental model see `MENTAL_MODEL.md`; to extend the engine see `BUILDING_ROBOTS.md`.

## The goal
Extract the **physics-robotics core** of [lpzrobots](https://github.com/georgmartius/lpzrobots) into a
small, standalone C++ engine built directly on the **Open Dynamics Engine (ODE)** — keeping the way
lpzrobots builds robots, dropping everything else (OpenSceneGraph rendering, the `selforg`
controllers/matrix lib, Qt, GUI, threading). **Only external dependency: `libode`.**

## Architecture (the extracted core)
```
OdeHandle   world + collision space + contact-joint group + default material + sub-spaces + ignored pairs
Primitive   a dBody (mass/motion) + a dGeom (shape/collision) welded by dGeomSetBody; Box/Sphere/Capsule/
            Cylinder/Plane/Ray/Transform
Joint       Fixed/Hinge/Hinge2/Universal/Ball/Slider — thin wrappers over dJointCreate*/Attach/anchor/axis
Substance   intuitive material (roughness/slip/hardness/elasticity) -> ODE surface (mu, ERP/CFM) by the
            spring-damper law; presets + anisotropic friction
Simulation  ODE lifecycle + the canonical loop: dSpaceCollide -> nearCallback -> dWorldStep -> dJointGroupEmpty
OdeRobot    a bag of Primitives+Joints + aggregating sensor/motor I/O (addSensor/addMotor)
servo/PID, RaySensor, JointSensor, obstacles, Logger   the reusable control/sensing/observability layer
robots/     Nimm4 (differential drive), Arm (servo chain), Snake (anisotropic-friction undulation)
```

## Key engineering decisions (and why)
- **One shape per body + joints for inertia; offset `Transform` geoms are collision-only (massless).**
  An attempt to compose offset masses (parallel-axis) was reverted: ODE 0.16 hard-requires the COM at
  the body origin, so incremental composition is fragile (a 2nd offset part double-moves the body) and
  it breaks lpzrobots' own bumper pattern. The faithful design is correct *and* robust. (`getShapeMass`
  is kept as a utility for manual, single-body composites.)
- **Position control uses a velocity servo (`OneAxisServoVel`), not a force PID.** A torque-PID against
  a fixed-timestep integrator limit-cycles at high gain; feeding a target *velocity* to the joint's
  implicit ODE motor is unconditionally stable and settles with ~zero steady-state error.
- **Robots are active agents → never auto-disable.** Their bodies set `dBodySetAutoDisableFlag(...,0)`
  (a sleeping body ignores joint motors); the world default auto-disable still rests passive objects.
- **Composable I/O (the lpzrobots ergonomic, restored).** `OdeRobot` aggregates its own `*Intern`
  channels with attached `Sensor`/`Motor` objects, so a new robot gets ray/joint sensing by
  `addSensor(...)` without rewriting its interface.
- **Guards on ODE's sharp edges:** parallel Universal/Hinge2 axes (would abort ODE), wrong-order
  teardown vs `dCloseODE` (lifetime guard), nested Transforms, and real-pointer ignored-pair keys.
- **Math layer** reproduces OpenSceneGraph's row-vector convention (`v·M`, w-first quaternions) so the
  ODE↔matrix bridge is correct — validated by round-tripping a pose through ODE itself.

## What is proven (not just "it ran")
Every claim is asserted by a headless self-checking test (`make test`, 7 suites, also CMake/`ctest`):
- math conventions round-trip exactly through ODE; energy dissipates (never grows); hinge/universal/
  ball/slider constraints hold; stacks are stable; no NaN over long runs.
- single-shape inertia tensors match closed form to machine precision; Coulomb friction slips at
  `atan(mu)`; restitution rises monotonically with elasticity with **no energy gain**; momentum &
  angular momentum conserved; **bit-exact determinism**; clean SI units (F=ma, tau=I*alpha exact).
- robots behave: Nimm4 drives/turns; Arm tracks position commands; Snake crawls ≈7.5 m via
  anisotropic friction; an attached ray reads true wall distance through the robot API.
- clean under AddressSanitizer + UBSan; only `<ode/ode.h>` is non-standard.

## Status & intentionally out of scope
Complete and verified as a foundation. Out of scope by choice (to stay small): rendering, a controller
class hierarchy (`Simulation::run` takes a `std::function`), trimesh/heightfield geoms, a separate
`AMotor` wrapper (the velocity servo uses the joint's built-in motor), and the lpzrobots learning
controllers (`selforg`).
