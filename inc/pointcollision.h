#ifndef POINTCOLLISION_H
#define POINTCOLLISION_H

#include <float.h>

#include "point.h"

typedef struct { 
  Point *pPoint1, *pPoint2;
  Vector normal;
  double e;
  double penetration;
} PointCollision;

//PointCollision *pointCollisions;
//int collisionsAmount;

void resolvePointVelocity(Point *pPoint1, Point *pPoint2, Vector normal, double e, double dTime);

void resolvePointInterpenetration(Point *pPoint1, Point *pPoint2, Vector normal, double penetration);

void resolvePointCollision(PointCollision *pCollision, double dTime);

//void resolvePointCollisions(double dTime);

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
