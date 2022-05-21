#ifndef COLLISION_H
#define COLLISION_H

#include <float.h>

#include "point.h"

typedef struct { 
  Point *pPoint1, *pPoint2;
  Vector normal;
  double e;
  double penetration;
} PointCollision;

PointCollision *pointCollisions;
int collisionsAmount;

void resolveVelocity(Point *pPoint1, Point *pPoint2, Vector normal, double e, double dTime) {
  double separatingVelocity = scalarProd(vSub(pPoint1->v, pPoint2->v), normal);
  if (separatingVelocity > 0) return;
  double newSeparatingVelocity = -separatingVelocity * e;

  Vector acv = vSub(pPoint1->a, pPoint2->a);
  double acsv = scalarProd(normal, acv) * dTime;
  if (acsv < 0) {
    newSeparatingVelocity += e * acsv;
    if (newSeparatingVelocity < 0) newSeparatingVelocity = 0;
  }

  double deltaVelocity = newSeparatingVelocity - separatingVelocity;
  double totalInverseMass = pPoint1->inverseMass + pPoint2->inverseMass;
  Vector impulsePerIMass = vMult(normal, deltaVelocity / totalInverseMass);
  pPoint1->v = vAdd(pPoint1->v, vMult(impulsePerIMass, pPoint1->inverseMass));
  pPoint2->v = vSub(pPoint2->v, vMult(impulsePerIMass, pPoint2->inverseMass));
}

void resolveInterpenetration(Point *pPoint1, Point *pPoint2, Vector normal, double penetration) {
  if (penetration <= 0) return;
  double totalInverseMass = pPoint1->inverseMass + pPoint2->inverseMass;
  Vector movePerIMass = vMult(normal, penetration / totalInverseMass);
  pPoint1->p = vAdd(pPoint1->p, vMult(movePerIMass, pPoint1->inverseMass));
  pPoint2->p = vSub(pPoint2->p, vMult(movePerIMass, pPoint2->inverseMass));
}

void resolveCollision(PointCollision *pCollision, double dTime) {
  resolveVelocity(pCollision->pPoint1, pCollision->pPoint2, pCollision->normal, pCollision->e, dTime);
  resolveInterpenetration(pCollision->pPoint1, pCollision->pPoint2, pCollision->normal, pCollision->penetration);
}

void resolveCollisions(double dTime) {
  for (int i = 0; i < collisionsAmount; i++) {
    resolveCollision(&pointCollisions[i], dTime);
  }
}

/*void resolveCollisions(PointCollision *pCollisionsArray, double dTime) {
  printf("loop start; collisions = %d\n", collisionsAmount);
  for (int iterC = 0; iterC < collisionsAmount; iterC++) {
    double max = DBL_MAX;
    int maxIndex = collisionsAmount;
    for (int i = 0; i < collisionsAmount; i++) {
      double separatingVelocity = scalarProd(vSub(pCollisionsArray[i].pPoint1->v, pCollisionsArray[i].pPoint2->v), pCollisionsArray[i].normal);
      if (separatingVelocity < max && (separatingVelocity <= 0 || pCollisionsArray[i].penetration >= 0)) {
        max = separatingVelocity;
        maxIndex = i;
      }
    }
    if (maxIndex == collisionsAmount) {printf("ok\n"); break;}
    printf("coll sepVel %7.3f\n", scalarProd(vSub(pCollisionsArray[maxIndex].pPoint1->v, pCollisionsArray[maxIndex].pPoint2->v), pCollisionsArray[maxIndex].normal));
    printf("coll pen %7.3f\n", pointCollisions[maxIndex].penetration);
    printf("res coll %d\n", maxIndex);
    resolveCollision(&pCollisionsArray[maxIndex], dTime);
  }
  printf("loop end\n");
}*/

#endif
