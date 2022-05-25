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

void updateRigidbodySpringForces(RigidbodySpring *pRBS) {
  Vector lws = localToWorld(pRBS->p1, pRBS->pRB1->transformMatrix);
  Vector ows = localToWorld(pRBS->p2, pRBS->pRB2->transformMatrix);
  Vector force = vSub(ows, lws);
  printV(ows);
  //printV(force);

  Vector springForce = vMult(vNormalize(force), fabs(vLength(force) - pRBS->l) * pRBS->k);

  applyForceAtPoint(pRBS->pRB1,       springForce,      lws);
  applyForceAtPoint(pRBS->pRB2, vMult(springForce, -1), ows);
}

#endif
