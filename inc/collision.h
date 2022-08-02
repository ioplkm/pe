#include "vector.h"
#include "narrowcollision.h"

typedef struct {
  Rigidbody *pRB;
  Vector offset;
  Vector halfSize;
} ConvexPolyhedra;

double penetrationOnAxis(ConvexPolyhedra *pP1, ConvexPolyhedra *pP2, Vector axis);

int collison(ConvexPolyhedra *pP1, ConvexPolyhedra *pP2, Collision *pC);
