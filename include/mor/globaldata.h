/* mini_ode_robots — globaldata.h
 *
 * A *minimal* stand-in for lpzrobots' utils/globaldata.h. The full version
 * carries agents, obstacles, controllers, GUI hooks, etc. For a dependency-free
 * physics engine we keep only what the physics needs: the tunable simulation
 * parameters (the ODE skill's "tokens") and the current time.
 */
#ifndef MOR_GLOBALDATA_H
#define MOR_GLOBALDATA_H

namespace mor {

/// The simulation's tunable physics parameters — ODE skill: references/tokens.md
struct OdeConfig {
  double simStepSize   = 0.01;  ///< integration timestep (s). Fixed timestep (skill rule #5).
  double gravity       = -9.81; ///< gravity Z-component (the common case)
  double gravityX      = 0.0;   ///< gravity X (set together with the others by setGravity(Vec3))
  double gravityY      = 0.0;   ///< gravity Y
  int    controlInterval = 1;   ///< call the controller every N physics steps
  bool   useQuickStep  = false; ///< dWorldStep (accurate LCP) vs dWorldQuickStep (fast, iterative).
                                ///< Prefer QuickStep for large or perfectly-aligned stacks: the
                                ///< direct LCP solver emits recoverable 's<=0' warnings on singular
                                ///< aligned contacts, and QuickStep settles them more cleanly.
  int    quickStepIters= 20;    ///< iterations for QuickStep
  double cfm           = 1e-6;  ///< global constraint force mixing (soft global)
  double erp           = 0.2;   ///< global error reduction parameter
  bool   autoDisable   = true;  ///< let resting bodies sleep (good for debris/stacks). Set false
                                ///< for free-spin / energy-conservation experiments so a coasting
                                ///< body is not frozen. (Robots disable this per-body regardless.)
  // Scaling note: soft-contact penetration ~ m*g / kp, with kp = 100*h1*h2/(h1+h2) from the
  // colliding Substances. For heavy/dense bodies raise hardness (Metal h=200 => kp up to ~1e4)
  // or shrink simStepSize. Verified stable for geometry from ~1e-3 m to ~1e2 m at the default dt.
};

class GlobalData {
public:
  OdeConfig odeConfig;
  double    time = 0.0;
  long      sim_step = 0;   ///< integer step counter (used by sensors to dedupe per-step queries)
};

} // namespace mor
#endif
