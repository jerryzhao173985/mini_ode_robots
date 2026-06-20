/* mini_ode_robots — heightfield.h
 *
 * Rough-terrain ground: an ODE heightfield (dCreateHeightfield) wrapped as a static
 * Primitive so it collides with robots through the normal pipeline (dGeomSetData ->
 * Primitive -> Substance). Extracted from the ODE skill's geom-heightfield card +
 * lpzrobots terrainground.
 *
 * ODE grounding (the sharp edges, handled here):
 *  - Height runs along the geom's LOCAL Y axis (field in local X-Z). Our world is Z-up,
 *    so we make the geom placeable and rotate +90deg about X: local Y -> world Z,
 *    local Z -> world -Y. Hence a world point (X,Y) samples local (x=X, z=-Y).
 *  - The dHeightfieldDataID must OUTLIVE the geom: we own it and destroy the geom first.
 *  - Bounds must be set BEFORE binding (fixes the broadphase AABB).
 *
 * Usage:
 *   HeightField terrain(8, 8, 64, 64, [](double x,double y){ return 0.04*std::sin(1.5*x)*std::cos(1.5*y); });
 *   terrain.init(sim.getOdeHandle());
 *   // place a robot ON it at world (X,Y):  z = terrain.heightAt(X,Y) + robot.restHeight();
 */
#ifndef MOR_HEIGHTFIELD_H
#define MOR_HEIGHTFIELD_H

#include <ode/ode.h>
#include <functional>
#include "primitive.h"
#include "odehandle.h"
#include "pose_types.h"

namespace mor {

class HeightField : public Primitive {
public:
  /// @param width,depth   terrain extent in world X,Y (centred at the origin)
  /// @param wSamples,dSamples grid resolution
  /// @param heightFn      world (x,y) -> height; @param minH,maxH height bounds (for the AABB)
  HeightField(double width, double depth, int wSamples, int dSamples,
              std::function<double(double,double)> heightFn,
              double minH = -1.0, double maxH = 1.0)
    : width(width), depth(depth), wSamples(wSamples), dSamples(dSamples),
      heightFn(std::move(heightFn)), minH(minH), maxH(maxH) {}

  ~HeightField() override {
    // destroy the geom FIRST, then the data it referenced (lifecycle requirement)
    if (odeIsAlive()){
      if (geom){ dGeomDestroy(geom); geom = 0; }   // geom=0 so the base dtor skips it
      if (data){ dGeomHeightfieldDataDestroy(data); data = 0; }
    }
  }

  /// world (x,y) -> terrain height (use to place bodies ON the terrain)
  double heightAt(double x, double y) const { return heightFn(x, y); }

  void init(const OdeHandle& h, double /*mass*/, char /*mode*/ = Geom) override {
    if (!substanceManuallySet) substance = h.substance;
    mode = Geom;                                   // terrain is always static collision geometry
    data = dGeomHeightfieldDataCreate();
    dGeomHeightfieldDataBuildCallback(
        data, this, &HeightField::cb,
        width, depth, wSamples, dSamples,
        1.0 /*scale*/, 0.0 /*offset*/, 1.0 /*thickness: cuboid below to stop tunnelling*/, 0 /*wrap*/);
    dGeomHeightfieldDataSetBounds(data, minH, maxH);   // before binding
    geom = dCreateHeightfield(h.space, data, 1 /*placeable*/);
    attachGeomAndSetColliderFlags();
    // local Y (height) -> world Z : rotate +90deg about X
    dMatrix3 R; dRFromAxisAndAngle(R, 1, 0, 0, M_PI/2);
    dGeomSetRotation(geom, R);
  }

  void setMass(double, bool) override {}   // static, no mass

private:
  // ODE callback: grid (x,z) -> height. Map grid -> local, local -> world, eval heightFn.
  static dReal cb(void* userdata, int x, int z){
    HeightField* hf = static_cast<HeightField*>(userdata);
    double lx = ((double)x/(hf->wSamples-1) - 0.5) * hf->width;   // local x
    double lz = ((double)z/(hf->dSamples-1) - 0.5) * hf->depth;   // local z
    return (dReal) hf->heightFn(lx, -lz);                          // world (X,Y) = (lx, -lz)
  }

  double width, depth;
  int    wSamples, dSamples;
  std::function<double(double,double)> heightFn;
  double minH, maxH;
  dHeightfieldDataID data = nullptr;
};

} // namespace mor
#endif
