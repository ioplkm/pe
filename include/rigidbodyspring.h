#ifndef RIGIDBODYSPRING_H
#define RIGIDBODYSPRING_H

#include <math.h>

#include "rigidbody.h"
#include "matrix.h"

typedef struct {
  Vector p1, p2;
  Rigidbody *pRB1, *pRB2;
  double k, l;
} RigidbodySpring;

void updateRigidbodySpringForces(RigidbodySpring *pS) {
  Vector p1 = localToWorld(pS->p1, pS->pRB1->transformMatrix);
  Vector p2 = localToWorld(pS->p2, pS->pRB2->transformMatrix);
  Vector p1p2Vector = vSub(p2, p1);

  Vector springForce = vMult(vNormalize(p1p2Vector), (vLength(p1p2Vector) - pS->l) * pS->k);

  applyForceAtPoint(pS->pRB1, springForce, p1);
  applyForceAtPoint(pS->pRB2, vMult(springForce,  -1), p2);
}

#endif
