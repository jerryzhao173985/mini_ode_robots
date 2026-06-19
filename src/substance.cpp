/* mini_ode_robots — substance.cpp
 * The contact-material physics, extracted from lpzrobots osg/substance.cpp.
 * The combination formulas and preset values are reproduced exactly.
 */
#include "mor/substance.h"
#include "mor/globaldata.h"
#include "mor/pose_types.h"   // Axis, poseOfGeom, Pos
#include <cstdio>

namespace mor {

Substance::Substance() : callback(0), userdata(0) { toDefaultSubstance(); }

Substance::Substance(float roughness, float slip, float hardness, float elasticity)
  : roughness(roughness), slip(slip), hardness(hardness), elasticity(elasticity),
    callback(0), userdata(0) {}

void Substance::setCollisionCallback(CollisionCallback func, void* ud){ callback=func; userdata=ud; }

// Combination of two surfaces — THE contact physics. (lpzrobots substance.cpp:54)
void Substance::getSurfaceParams(dSurfaceParameters& sp, const Substance& s1, const Substance& s2,
                                 double stepsize){
  sp.mu = s1.roughness * s2.roughness;                                   // friction multiplies
  dReal  kp  = 100 * s1.hardness * s2.hardness / (s1.hardness + s2.hardness); // springs in series
  double kd1 = (1.00 - s1.elasticity);
  double kd2 = (1.00 - s2.elasticity);
  dReal  kd  = 50 * (kd1 * s2.hardness + kd2 * s1.hardness) / (s1.hardness + s2.hardness);
  sp.soft_erp = stepsize * kp / (stepsize * kp + kd);                    // standard ERP mapping
  sp.soft_cfm = 1 / (stepsize * kp + kd);                                // standard CFM mapping

  sp.slip1 = s1.slip + s2.slip;
  sp.slip2 = s1.slip + s2.slip;
  if (sp.slip1 < 0.0001) sp.mode = 0;
  else                   sp.mode = dContactSlip1 | dContactSlip2;
  sp.mode |= dContactSoftERP | dContactSoftCFM | dContactApprox1;
}

void Substance::printSurfaceParams(const dSurfaceParameters& sp){
  std::printf("Surface: mu=%g erp=%g cfm=%g slip=%g\n", sp.mu, sp.soft_erp, sp.soft_cfm, sp.slip1);
}

/* ----- factory presets (identical to lpzrobots) ----- */
Substance Substance::getDefaultSubstance(){ Substance s; s.toDefaultSubstance(); return s; }
void Substance::toDefaultSubstance(){ toPlastic(0.8); }

Substance Substance::getMetal(float r){ Substance s; s.toMetal(r); return s; }
void Substance::toMetal(float r){ roughness=r; hardness=200; elasticity=0.8; slip=0.01; }

Substance Substance::getRubber(float h){ Substance s; s.toRubber(h); return s; }
void Substance::toRubber(float h){ roughness=3; hardness=h; elasticity=0.95; slip=0.0; }

Substance Substance::getPlastic(float r){ Substance s; s.toPlastic(r); return s; }
void Substance::toPlastic(float r){ roughness=r; hardness=40; elasticity=0.5; slip=0.01; }

Substance Substance::getFoam(float h){ Substance s; s.toFoam(h); return s; }
void Substance::toFoam(float h){ roughness=2; hardness=h; elasticity=0; slip=0.1; }

Substance Substance::getSnow(float slip_){ Substance s; s.toSnow(slip_); return s; }
void Substance::toSnow(float slip_){ roughness=1.0-slip_; hardness=40; elasticity=0; slip=slip_; }

static int dummyCallBack(dSurfaceParameters&, GlobalData&, void*, dContact*, int,
                         dGeomID, dGeomID, const Substance&, const Substance&){ return 0; }

Substance Substance::getNoContact(){ Substance s; s.toNoContact(); return s; }
void Substance::toNoContact(){ toDefaultSubstance(); setCollisionCallback(dummyCallBack, 0); }

/* ----- anisotropic friction (snake scales): different friction along an axis ----- */
struct AnisotropFrictionData { double ratio; Axis axis; };

static int anisocallback(dSurfaceParameters& params, GlobalData& globaldata, void* userdata,
                         dContact* contacts, int numContacts,
                         dGeomID o1, dGeomID /*o2*/, const Substance& s1, const Substance& s2){
  if (s2.callback) return 1;  // the other side handles it
  AnisotropFrictionData* data = static_cast<AnisotropFrictionData*>(userdata);
  Pose pose = poseOfGeom(o1);
  Pos objectaxis = data->axis * pose;             // axis in world coords
  for (int i=0;i<numContacts;i++){
    Pos normal(contacts[i].geom.normal);
    Pos dir = Pos(objectaxis ^ normal);
    if (dir.isNaN() || dir.length2() < 0.1) return 1;     // perpendicular -> normal friction
    dir.normalize();
    contacts[i].fdir1[0]=dir.x(); contacts[i].fdir1[1]=dir.y(); contacts[i].fdir1[2]=dir.z();
  }
  Substance::getSurfaceParams(params, s1, s2, globaldata.odeConfig.simStepSize);
  params.mu2 = params.mu * data->ratio;
  params.mode |= dContactMu2 | dContactFDir1;
  return 2;
}

void Substance::toAnisotropFriction(double ratio, const Axis& axis){
  AnisotropFrictionData* data = new AnisotropFrictionData{ratio, axis}; // (intentional small leak, as upstream)
  setCollisionCallback(anisocallback, data);
}

} // namespace mor
