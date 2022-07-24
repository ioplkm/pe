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
  double desv = scalarProd(dvWorld, pC->normal);
  desv += pC->pB1->inverseMass;

  Vector localP2 = vSub(pC->p, pC->pB2->p);
  dvWorld = vectorProd(localP2, pC->normal);
  dvWorld = m33vMult(pC->pB2->iit, dvWorld);
  dvWorld = vectorProd(dvWorld, localP2);
  //Vector vPerUnitImpulseCollision = m33vMult(m33transpose(transform), vPerUnitImpulse);
  //double v = vPerUnitImpulseCollision.x;
  desv = scalarProd(dvWorld, pC->normal);
  desv += pC->pB2->inverseMass;

  Vector cv = vectorProd(pC->pB1->r, localP1);
  cv = vAdd(cv, pC->pB1->v);
  cv = vSub(cv, vectorProd(pC->pB2->r, localP2));
  cv = vSub(cv, pC->pB2->v);
  cv = m33vMult(m33Transpose(transform), cv);
  double dv = -cv.x * (1 + pC->r);
}
