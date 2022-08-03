#ifndef POINTSPRING_H
#define POINTSPRING_H

#include "point.h"

typedef struct {
  Point *pPoint1, *pPoint2;
  double l, k;
} Spring;

typedef struct {
  Point *pPoint;
  Vector anchor;
  double l, k;
} anchoredSpring;

void updateSpringForces(Spring s);

void updateAnchoredSpringForces(anchoredSpring s);

#endif
