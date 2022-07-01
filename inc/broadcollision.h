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
  struct BVHNode *parent;
  BoundingVolume volume;
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
  if (!isOverlap(&pN1->volume, &pN2->volume)) return;
  if (pN1->pRB && pN2->pRB) {
    potentialCollisions[c].pRB1 = pN1->pRB;
    potentialCollisions[c++].pRB2 = pN2->pRB;
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

void recalcVolume(BVHNode *pNode) {
  pNode->volume = bvFrom2BV(&pNode->pC1->volume, &pNode->pC2->volume);
  if (pNode->parent) recalcVolume(pNode->parent);
}

double getGrowth(BoundingVolume *pV, BoundingVolume *opV) {
  BoundingVolume newV = bvFrom2BV(pV, opV);
  return (newV.r*newV.r - pV->r*pV->r);
}

void insertToBVH(BVHNode *pNode, Rigidbody *pRB, BoundingVolume *pV) {
  if (pNode->pRB) {
    pNode->pC1 = (BVHNode*)malloc(sizeof(BVHNode));
    pNode->pC1->parent = pNode;
    pNode->pC1->volume = pNode->volume;
    pNode->pC1->pRB = pNode->pRB;
    pNode->pC1->pC1 = NULL;
    pNode->pC1->pC2 = NULL;

    pNode->pC2 = (BVHNode*)malloc(sizeof(BVHNode));
    pNode->pC2->parent = pNode;
    pNode->pC2->volume = *pV;
    pNode->pC2->pRB = pRB;
    pNode->pC2->pC1 = NULL;
    pNode->pC2->pC2 = NULL;

    pNode->pRB = NULL;
    recalcVolume(pNode);
  } else {
    getGrowth(&pNode->pC1->volume, pV) < getGrowth(&pNode->pC2->volume, pV) ?
      insertToBVH(pNode->pC1, pRB, pV) :
      insertToBVH(pNode->pC2, pRB, pV);
  }
}

void deleteFromBVH(BVHNode *pNode) {
  BVHNode *sibling = (pNode->parent->pC1 == pNode) ? pNode->parent->pC2 : pNode->parent->pC1;
  pNode->parent->volume = sibling->volume;
  pNode->parent->pRB = sibling->pRB;
  pNode->parent->pC1 = sibling->pC1;
  pNode->parent->pC2 = sibling->pC2;

  recalcVolume(pNode->parent);
  free(sibling);
  free(pNode->pC1);
  free(pNode->pC2);
  free(pNode);
}

#endif
