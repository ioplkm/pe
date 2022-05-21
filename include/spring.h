#ifndef SPRING_H
#define SPRING_H

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

void updateSpringForces(Spring s) {
  if (vIsEqual(s.pPoint1->p, s.pPoint2->p)) return;
  Vector point1point2Vector = vSub(s.pPoint2->p, s.pPoint1->p);
  Vector springForce = vMult(vNormalize(point1point2Vector), (vLength(point1point2Vector) - s.l) * s.k);
  s.pPoint1->f = vAdd(s.pPoint1->f, springForce);
  s.pPoint2->f = vSub(s.pPoint2->f, springForce);
}

void updateAnchoredSpringForces(anchoredSpring s) {
  if (vIsEqual(s.anchor, s.pPoint->p)) return;
  Vector pointAnchorVector = vSub(s.anchor, s.pPoint->p);
  Vector springForce = vMult(vNormalize(pointAnchorVector), (vLength(pointAnchorVector) - s.l) * s.k);
  s.pPoint->f = vAdd(s.pPoint->f, springForce);
}

#endif
