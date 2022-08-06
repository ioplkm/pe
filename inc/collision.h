#ifndef COLLISION_H
#define COLLISION_H

#include "vector.h"
#include "narrowcollision.h"

typedef struct {
  Rigidbody *pRB;
  Vector offset;
  Vector halfSize;
} ConvexPolyhedra;

double penetrationOnAxis(ConvexPolyhedra *pP1, ConvexPolyhedra *pP2, Vector axis);

int collision(ConvexPolyhedra *pP1, ConvexPolyhedra *pP2, Collision *pC);

#endif
