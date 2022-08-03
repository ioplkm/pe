#ifndef CABLE_H
#define CABLE_H

#include "pointcollision.h"

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

void createCollisionFromCable(Cable *pCable, PointCollision *pC);

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

void createCollisionFromRod(Cable *pCable, PointCollision *pC);

#endif
