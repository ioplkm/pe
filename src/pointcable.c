#include "../inc/pointcable.h"

void createCollisionFromCable(Cable *pCable, PointCollision *pC) {
  double currentLength = vDist(pCable->pPoint1->p, pCable->pPoint2->p);
  if (currentLength < pCable->maxLength) return ; //no collision
  pC->pPoint1 = pCable->pPoint1;
  pC->pPoint2 = pCable->pPoint2;
  pC->normal = vNormalize(vSub(pCable->pPoint2->p, pCable->pPoint1->p));
  pC->penetration = currentLength - pCable->maxLength;
  pC++->e = pCable->bounciness;
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

void createCollisionFromRod(Cable *pCable, PointCollision *pC) {
  double currentLength = vDist(pCable->pPoint1->p, pCable->pPoint2->p);
  //if (currentLength == pCable->maxLength) return; //no collision
  pC->pPoint1 = pCable->pPoint1;
  pC->pPoint2 = pCable->pPoint2;
  if (currentLength > pCable->maxLength) {
    pC->normal = vNormalize(vSub(pCable->pPoint2->p, pCable->pPoint1->p));
    pC->penetration = currentLength - pCable->maxLength;
  } else {
    pC->normal = vNormalize(vSub(pCable->pPoint1->p, pCable->pPoint2->p));
    pC->penetration = pCable->maxLength - currentLength;
  }
  pC++->e = 0;
}
