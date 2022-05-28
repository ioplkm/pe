#ifndef BROADCOLLISION_H
#define BROADCOLLISION_H

#include <math.h>
#include <stdbool.h>

#include "vector.h"
#include "rigidbody.h"

typedef struct {
  Vector p;
  double r;
} BoundingVolume;

typedef struct {
  Rigidbody *pRB1, *pRB2;
} PotentialCollision;

typedef struct BVHNode {
  BoundingVolume *volume;
  Rigidbody *pRB;
  struct BVHNode *pC1, *pC2;
} BVHNode;

PotentialCollision potentialCollisions[9999];
int c;

BoundingVolume bvFrom2BV(BoundingVolume *pV1, BoundingVolume *pV2) {
  double dist = vDist(pV1->p, pV2->p);
  if (!dist) return (BoundingVolume){pV1->p, pV1->r > pV2->r ? pV1->r : pV2->r};
  double rad = (dist + pV1->r + pV2->r)/2.0;
  return (BoundingVolume){vAdd(pV1->p, vMult(vSub(pV2->p, pV1->p), (rad - pV1->r) / dist)), rad};
}

bool isOverlap(BoundingVolume *pV1, BoundingVolume *pV2) {
  return vDist2(pV1->p, pV2->p) < pow(pV1->r + pV2->r, 2);
}

void isPotentialContact(BVHNode *pN1, BVHNode *pN2) {
  if (!isOverlap(pN1->volume, pN2->volume)) return;
  if (pN1->pRB && pN2->pRB) {
    potentialCollisions[c].pRB1 = pN1->pRB;
    potentialCollisions[++c].pRB2 = pN2->pRB;
    return;
  }
  if (pN1->pRB)
    isPotentialContact(pN2->pC1, pN2->pC2);
  if (pN2->pRB) 
    isPotentialContact(pN1->pC1, pN1->pC2);
}

void getPotentialContacts(BVHNode *pRoot) {
  isPotentialContact(pRoot->pC1, pRoot->pC2);
}

#endif
