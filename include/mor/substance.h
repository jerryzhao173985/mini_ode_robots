/* mini_ode_robots — substance.h
 *
 * The physically-parameterized contact material model, extracted verbatim from
 * lpzrobots' osg/substance.{h,cpp}. Four intuitive scalars replace ODE's raw
 * dSurfaceParameters (mu / ERP / CFM); two materials combine with real physics:
 *
 *   mu   = roughness1 * roughness2
 *   kp   = 100 * h1*h2/(h1+h2)                 (two springs in series)
 *   kd   = 50  * ((1-e1)*h2 + (1-e2)*h1)/(h1+h2)
 *   ERP  = h*kp / (h*kp + kd)                  (standard spring-damper -> ERP/CFM)
 *   CFM  = 1    / (h*kp + kd)                  (h = timestep)
 *
 * Ground truth: lpzrobots substance.cpp:54; ODE skill foundations/erp-cfm-friction.md.
 */
#ifndef MOR_SUBSTANCE_H
#define MOR_SUBSTANCE_H

#include <ode/ode.h>

namespace mor {

class GlobalData;
class Substance;
class Axis;

/** Per-collision callback (stored in a Substance).
    @return 0 = ignore this collision; 1 = use default surface params;
            2 = params already set by the callback. */
typedef int (*CollisionCallback)(dSurfaceParameters& params, GlobalData& globaldata, void* userdata,
                                 dContact* contacts, int numContacts,
                                 dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2);

class Substance {
public:
  float roughness;
  float slip;
  float hardness;
  float elasticity;

  CollisionCallback callback;
  void*             userdata;   ///< NON-OWNING: Substance is value-copied freely; never freed in any dtor.
                                ///< toAnisotropFriction() allocates a small block here that is intentionally
                                ///< leaked (as in upstream lpzrobots) — use it for one-shot setup only.

  Substance();                                                  // default: plastic(0.8)
  Substance(float roughness, float slip, float hardness, float elasticity);

  void setCollisionCallback(CollisionCallback func, void* userdata);

  /// combine two materials into ODE surface params (the core physics)
  static void getSurfaceParams(dSurfaceParameters& sp, const Substance& s1, const Substance& s2,
                               double stepsize);
  static void printSurfaceParams(const dSurfaceParameters& sp);

  /* --- factory presets (identical ranges/values to lpzrobots) --- */
  static Substance getDefaultSubstance();  void toDefaultSubstance();
  static Substance getMetal(float roughness);    void toMetal(float roughness);
  static Substance getRubber(float hardness);    void toRubber(float hardness);
  static Substance getPlastic(float roughness);  void toPlastic(float roughness);
  static Substance getFoam(float hardness);      void toFoam(float hardness);
  static Substance getSnow(float slip);          void toSnow(float slip);
  static Substance getNoContact();               void toNoContact();

  /// anisotropic friction along an axis (e.g. snake scales). Sets the callback.
  void toAnisotropFriction(double ratio, const Axis& axis);
};

} // namespace mor
#endif
