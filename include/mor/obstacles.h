/* mini_ode_robots — obstacles.h
 *
 * Minimal passive scene objects, in the spirit of lpzrobots obstacles/.
 * An obstacle is just Primitive(s): a Playground is four STATIC wall boxes
 * (geom, no body); a PassiveBox/PassiveSphere is a normal dynamic primitive.
 * Header-only — they add nothing beyond Primitive.
 */
#ifndef MOR_OBSTACLES_H
#define MOR_OBSTACLES_H

#include <vector>
#include "primitive.h"
#include "odehandle.h"

namespace mor {

/// A square arena of four static walls centred at (cx,cy). Owns its wall boxes.
class Playground {
public:
  Playground(double sideLength = 8.0, double width = 0.2, double height = 0.6)
    : side(sideLength), wallW(width), wallH(height) {}
  ~Playground(){ for (auto* w : walls) delete w; }

  void init(const OdeHandle& oh, double cx = 0, double cy = 0){
    double h = side * 0.5;
    // (length, thickness) walls along +/-x and +/-y, static (Geom only)
    struct W { double x,y,lx,ly; };
    // all four walls are (side + wallW) long in their span direction, so the
    // corners meet with a single clean wallW x wallW overlap (no double thickness).
    W ws[4] = {
      { cx,      cy + h, side + wallW, wallW        },  // north
      { cx,      cy - h, side + wallW, wallW        },  // south
      { cx + h,  cy,     wallW,        side + wallW },  // east
      { cx - h,  cy,     wallW,        side + wallW },  // west
    };
    for (auto& w : ws){
      Box* b = new Box(w.lx, w.ly, wallH);
      b->init(oh, 0, Primitive::Geom);             // static wall
      b->setPosition(Pos(w.x, w.y, wallH * 0.5));
      walls.push_back(b);
    }
  }
  const std::vector<Primitive*>& getWalls() const { return walls; }

private:
  double side, wallW, wallH;
  std::vector<Primitive*> walls;
};

/// A dynamic box you can drop into a scene.
inline Box* makePassiveBox(const OdeHandle& oh, const Pos& pos, double size = 0.4, double mass = 1.0){
  Box* b = new Box(size, size, size);
  b->init(oh, mass);
  b->setPosition(pos);
  return b;
}
/// A dynamic sphere you can drop into a scene.
inline Sphere* makePassiveSphere(const OdeHandle& oh, const Pos& pos, double r = 0.3, double mass = 1.0){
  Sphere* s = new Sphere(r);
  s->init(oh, mass);
  s->setPosition(pos);
  return s;
}

} // namespace mor
#endif
