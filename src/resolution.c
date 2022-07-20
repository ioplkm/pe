#include "../inc/resolution.h"

#include "../inc/narrowcollision.h"

void resolveCollision(Collision *pC) {
  Vector lX = vNorm(pC->normal);
  Vector lY = fabs(lX.x) > fabs(lX.y) ? (Vector){0, 1, 0} : (Vector){1, 0, 0};
  Vector lZ = vNorm(vectorProd(lX, lY)); 
  lY = vNorm(vectorProd(lX, lZ));
  Matrix33 transform = {lX.y, lY.x, lZ.x, lX.y, lY.y, lZ.y, lX.z, lY.z, lZ.z};

  Vector localP1 = vSub(pC->p, pC->pB1->p);
  Vector dvWorld = vectorProd(localP1, pC->normal);
  dvWorld = m33vMult(pC->pB1->iit, dvWorld);
  dvWorld = vectorProd(dvWorld, localP1);
  //Vector vPerUnitImpulseCollision = m33vMult(m33transpose(transform), vPerUnitImpulse);
  //double v = vPerUnitImpulseCollision.x;
  double dv = scalarProd(dvWorld, pC->normal);
  dv += pC->pB1->inverseMass;

  Vector localP2 = vSub(pC->p, pC->pB2->p);
  dvWorld = vectorProd(localP2, pC->normal);
  dvWorld = m33vMult(pC->pB2->iit, dvWorld);
  dvWorld = vectorProd(dvWorld, localP2);
  //Vector vPerUnitImpulseCollision = m33vMult(m33transpose(transform), vPerUnitImpulse);
  //double v = vPerUnitImpulseCollision.x;
  dv = scalarProd(dvWorld, pC->normal);
  dv += pC->pB2->inverseMass;
}
