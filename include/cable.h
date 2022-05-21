#ifndef CABLE_H
#define CABLE_H

#include "collision.h"

typedef struct {
  Point *pPoint1, *pPoint2;
  double maxLength;
  double bounciness;
} Cable;

typedef struct {
  Point *pPoint;
  Vector anchor;
  double maxLength;
  double bounciness;
} AnchoredCable;

void createCollisionFromCable(Cable *pCable) {
  double currentLength = vDist(pCable->pPoint1->p, pCable->pPoint2->p);
  if (currentLength < pCable->maxLength) return ; //no collision
  pointCollisions[collisionsAmount].pPoint1 = pCable->pPoint1;
  pointCollisions[collisionsAmount].pPoint2 = pCable->pPoint2;
  pointCollisions[collisionsAmount].normal = vNormalize(vSub(pCable->pPoint2->p, pCable->pPoint1->p));
  pointCollisions[collisionsAmount].penetration = currentLength - pCable->maxLength;
  pointCollisions[collisionsAmount].e = pCable->bounciness;
  collisionsAmount++;
}

/*int createCollisionFromAnchoredCable(AnchoredCable *pAnchoredCable, PointCollision *pPointCollision) {
  double currentLength = vDist(pAnchoredCable->pPoint->p, pAnchoredCable->anchor);
  if (currentLength < pAnchoredCable->maxLength) return 0; //no collision
  pPointCollision->pPoint1 = pAnchoredCable->pPoint;
  pPointCollision->pPoint2 = &(Point){pAnchoredCable->anchor, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};
  Vector normal = vSub(pAnchoredCable->anchor, pAnchoredCable->pPoint->p);
  pPointCollision->normal = vNormalize(normal);
  pPointCollision->penetration = currentLength - pAnchoredCable->maxLength;
  pPointCollision->e = pAnchoredCable->bounciness;
  return 1;
}*/

void createCollisionFromRod(Cable *pCable) {
  double currentLength = vDist(pCable->pPoint1->p, pCable->pPoint2->p);
  //if (currentLength == pCable->maxLength) return; //no collision
  pointCollisions[collisionsAmount].pPoint1 = pCable->pPoint1;
  pointCollisions[collisionsAmount].pPoint2 = pCable->pPoint2;
  if (currentLength > pCable->maxLength) {
    pointCollisions[collisionsAmount].normal = vNormalize(vSub(pCable->pPoint2->p, pCable->pPoint1->p));
    pointCollisions[collisionsAmount].penetration = currentLength - pCable->maxLength;
  } else {
    pointCollisions[collisionsAmount].normal = vNormalize(vSub(pCable->pPoint1->p, pCable->pPoint2->p));
    pointCollisions[collisionsAmount].penetration = pCable->maxLength - currentLength;
  }
  pointCollisions[collisionsAmount].e = 0;
  collisionsAmount++;
}

#endif
