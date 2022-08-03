#ifndef POINT_H
#define POINT_H

#include "vector.h"

typedef struct {
  Vector p, v, a, f;
  double inverseMass;
} Point;

void updatePoint(Point *pPoint, double dTime);

#endif
