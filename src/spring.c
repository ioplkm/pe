#include "../inc/spring.h"

void updateRigidbodySpringForces(RigidbodySpring *pS) {
  Vector p1 = m34vMult(pS->pRB1->transform, pS->p1);
  Vector p2 = m34vMult(pS->pRB2->transform, pS->p2);
  Vector p1p2Vector = vSub(p2, p1);

  Vector springForce = vMult(vNormalize(p1p2Vector), (vLength(p1p2Vector) - pS->l) * pS->k);

  applyForceAtPoint(pS->pRB1, springForce, p1);
  applyForceAtPoint(pS->pRB2, vMult(springForce,  -1), p2);
}
