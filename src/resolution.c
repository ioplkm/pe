#include "../inc/resolution.h"

void resolveCollision(Collision *pC) {
  resolveVelocity(pC);
  resolveInterpenetration(pC);
}

void resolveVelocity(Collision *pC) {
  //creating orthonormal basis
  Vector lX = vNorm(pC->normal);
  Vector lY = fabs(lX.x) > fabs(lX.y) ? (Vector){0, 1, 0} : (Vector){1, 0, 0};
  Vector lZ = vNorm(vectorProd(lX, lY)); 
  lY = vNorm(vectorProd(lX, lZ));
  Matrix33 transform = {lX.x, lY.x, lZ.x, lX.y, lY.y, lZ.y, lX.z, lY.z, lZ.z};
  //calculating delta velocity
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
//  dv += scalarProd(dvWorld, pC->normal);
//  dv += pC->pB2->inverseMass;
  //calculating desired velocity and impulse
  Vector cv = vectorProd(pC->pB1->r, localP1);
  cv = vAdd(cv, pC->pB1->v);
//  cv = vSub(cv, vectorProd(pC->pB2->r, localP2));
//  cv = vSub(cv, pC->pB2->v);
  cv = m33vMult(m33Transpose(transform), cv);
  double desv = -cv.x * (1 + pC->r);
  Vector imp = {desv / dv, 0, 0};
  imp = m33vMult(transform, imp);
  //applying impulse
  pC->pB1->v = vAdd(pC->pB1->v, vMult(imp, pC->pB1->inverseMass));
  //pC->pB1->r = vAdd(pC->pB1->r, m33vMult(pC->pB1->iit, vectorProd(imp, localP1)));
//  pC->pB2->v = vAdd(pC->pB2->v, vMult(imp, -pC->pB2->inverseMass));
//  pC->pB2->r = vAdd(pC->pB2->r, m33vMult(pC->pB2->iit, vectorProd(vInv(imp), localP2)));
}

void resolveInterpenetration(Collision *pC) {
  Vector localP1 = vSub(pC->p, pC->pB1->p);
  Vector angInertia = vectorProd(localP1, pC->normal);
  angInertia = m33vMult(pC->pB1->iit, angInertia);
  angInertia = vectorProd(angInertia, localP1);
  double angInertia1 = scalarProd(angInertia, pC->normal);
  Vector localP2 = vSub(pC->p, pC->pB2->p);
  angInertia = vectorProd(localP2, pC->normal);
  angInertia = m33vMult(pC->pB2->iit, angInertia);
  angInertia = vectorProd(angInertia, localP2);
  double angInertia2 = scalarProd(angInertia, pC->normal);
  //double iInertia = 1 / (pC->pB1->inverseMass + pC->pB2->inverseMass + angInertia1 + angInertia2);
  double iInertia = 1 / (pC->pB1->inverseMass + angInertia1);
  //calculating linear and angular movements
  double linMove1 = pC->penetration * pC->pB1->inverseMass * iInertia;
  double angMove1 = pC->penetration * angInertia1 * iInertia;
  double limit1 = 0.2 * vLength(localP1);
  if (fabs(angMove1) > limit1) {
    double totalMove1 = angMove1 + linMove1;
    angMove1 = angMove1 >= 0 ? limit1 : -limit1;
    linMove1 = totalMove1 - angMove1;
  }
  double linMove2 = -pC->penetration * pC->pB2->inverseMass * iInertia;
  double angMove2 = -pC->penetration * angInertia2 * iInertia;
  double limit2 = 0.2 * vLength(localP2);
  if (fabs(angMove2) > limit2) {
    double totalMove2 = angMove2 + linMove2;
    angMove2 = angMove2 >= 0 ? limit2 : -limit2;
    linMove2 = totalMove2 - angMove2;
  }
  //applying linear movements
  pC->pB1->p = vAdd(pC->pB1->p, vMult(pC->normal, linMove1));
//  pC->pB2->p = vAdd(pC->pB2->p, vMult(pC->normal, linMove2));
  //applying angular movements
  Vector impulsePerMove = m33vMult(pC->pB1->iit, vectorProd(localP1, pC->normal));
  Vector rotation = vMult(impulsePerMove, angMove1 / angInertia1);
  //pC->pB1->o = qvAdd(pC->pB1->o, rotation);
  impulsePerMove = m33vMult(pC->pB2->iit, vectorProd(localP2, pC->normal));
  Vector rotationPerMove = vDiv(impulsePerMove, angInertia2);
  rotation = vMult(rotationPerMove, angMove2);
//  pC->pB2->o = qRotate(pC->pB2->o, rotation);
} 
