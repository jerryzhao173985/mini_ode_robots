/* mini_ode_robots — odehandle.cpp */
#include "mor/odehandle.h"
#include "mor/primitive.h"
#include <algorithm>

namespace mor {

// ODE lifecycle guard (see odehandle.h). A counter so nested Simulations work.
static int g_odeAlive = 0;
void odeLifecycleAcquire(){ ++g_odeAlive; }
void odeLifecycleRelease(){ if (g_odeAlive > 0) --g_odeAlive; }
bool odeIsAlive(){ return g_odeAlive > 0; }

OdeHandle::OdeHandle()
  : world(0), space(0), jointGroup(0), timePtr(0),
    ignoredPairs(std::make_shared<std::unordered_set<std::pair<dGeomID,dGeomID>, GeomPairHash>>()),
    spaces(std::make_shared<std::vector<dSpaceID>>()) {}

OdeHandle::OdeHandle(dWorldID w, dSpaceID s, dJointGroupID j)
  : world(w), space(s), jointGroup(j), timePtr(0),
    ignoredPairs(std::make_shared<std::unordered_set<std::pair<dGeomID,dGeomID>, GeomPairHash>>()),
    spaces(std::make_shared<std::vector<dSpaceID>>()) {}

void OdeHandle::addSpace(dSpaceID s){ spaces->push_back(s); }
void OdeHandle::removeSpace(dSpaceID s){
  auto it = std::find(spaces->begin(), spaces->end(), s);
  if (it != spaces->end()) spaces->erase(it);
}

void OdeHandle::createNewSimpleSpace(dSpaceID parentspace, bool ignore_inside_collisions){
  space = dSimpleSpaceCreate(parentspace);
  dSpaceSetCleanup(space, 0);
  if (!ignore_inside_collisions) addSpace(space);
}
void OdeHandle::createNewHashSpace(dSpaceID parentspace, bool ignore_inside_collisions){
  space = dHashSpaceCreate(parentspace);
  dSpaceSetCleanup(space, 0);
  if (!ignore_inside_collisions) addSpace(space);
}
void OdeHandle::deleteSpace(){
  removeSpace(space);
  dSpaceDestroy(space);
}

void OdeHandle::addIgnoredPair(dGeomID g1, dGeomID g2){
  ignoredPairs->insert(std::make_pair(g1,g2));
  ignoredPairs->insert(std::make_pair(g2,g1));
}
void OdeHandle::removeIgnoredPair(dGeomID g1, dGeomID g2){
  ignoredPairs->erase(std::make_pair(g1,g2));
  ignoredPairs->erase(std::make_pair(g2,g1));
}
void OdeHandle::addIgnoredPair(Primitive* p1, Primitive* p2){
  if (!p1->getGeom() || !p2->getGeom()) return;
  addIgnoredPair(p1->getGeom(), p2->getGeom());
}
void OdeHandle::removeIgnoredPair(Primitive* p1, Primitive* p2){
  if (!p1->getGeom() || !p2->getGeom()) return;
  removeIgnoredPair(p1->getGeom(), p2->getGeom());
}

} // namespace mor
