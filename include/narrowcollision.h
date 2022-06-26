#ifndef NARROWCOLLISION_H
#define NARROWCOLLISION_H

#include "vector.h"
#include "rigidbody.h"
#include "matrix.h"

/*typedef struct {
  Rigidbody *pRB;
  Matrix34 offset;
} Primitive;*/

typedef struct {
  Rigidbody *pRB;
  Matrix34 offset;
  double r;
} CollisionSphere;

typedef struct {
  Rigidbody *pRB;
  Matrix34 offset;
  Vector normal;
  double planeOffset;
} CollisionPlane;

typedef struct {
  Rigidbody *pRB;
  Matrix34 offset;
  Vector halfSize;
} CollisionBox;

typedef struct {
  Vector p;
  Vector normal;
  double penetration;
} Collision;

Collision collisions[999];
int collisionC;

void SphereSphereCollision(CollisionSphere *pCS1, CollisionSphere *pCS2) {
  Vector s1p = pCS1->pRB->p;
  Vector s2p = pCS2->pRB->p;
  Vector midline = vSub(s1p, s2p);
  double size = vLength(midline);

  collisions[collisionC].p = vAdd(s1p, vMult(midline, 0.5));
  collisions[collisionC].normal = vMult(midline, 1/size);
  collisions[collisionC].penetration = pCS1->r + pCS2->r - size;
  collisionC++;
}

void SphereHalfSpaceCollision(CollisionSphere *pCS, CollisionPlane *pCP) {
  Vector sp = pCS->pRB->p;
  double distance = scalarProd(pCP->normal, sp) - pCS->r - pCP->planeOffset;
  
  collisions[collisionC].p = vSub(sp, vMult(pCP->normal, distance + pCS->r));
  collisions[collisionC].normal = pCP->normal;
  collisions[collisionC].penetration = -distance;
  collisionC++;
}

void SpherePlaneCollision(CollisionSphere *pCS, CollisionPlane *pCP) {
  Vector sp = pCS->pRB->p;
  double distance = scalarProd(pCP->normal, sp) - pCP->planeOffset;

  collisions[collisionC].p = vSub(sp, vMult(pCP->normal, distance + pCS->r));
  collisions[collisionC].normal = distance < 0 ? vMult(pCP->normal, -1) : pCP->normal;
  collisions[collisionC].penetration = distance < 0 ? distance + pCS->r : -distance + pCS->r;
  collisionC++;
}

void BoxHalfSpaceCollision(CollisionBox *pCB, CollisionPlane *pCP) {
  for (int i = 0; i < 8; i++) {
    Vector p = {i & 1 ? -pCB->halfSize.x : pCB->halfSize.x,
                i & 2 ? -pCB->halfSize.y : pCB->halfSize.y,
                i & 4 ? -pCB->halfSize.z : pCB->halfSize.z};
    p = m34vMult(pCB->pRB->transformMatrix, p);
    double dist = scalarProd(p, pCP->normal);
    if (dist <= pCP->planeOffset) {
      collisions[collisionC].p = vAdd(vMult(pCP->normal, dist - pCP->planeOffset), p);
      collisions[collisionC].normal = pCP->normal;
      collisions[collisionC].penetration = pCP->planeOffset - dist;
    }
  }
}

void BoxSphereCollision(CollisionBox *pCB, CollisionSphere *pCS) {
  Vector relSphereCenter = worldToLocal(pCS->pRB->p, pCB->pRB->transformMatrix);
  Vector closestP = {relSphereCenter.x > pCB->halfSize.x ? pCB->halfSize.x : 
                       relSphereCenter.x < -pCB->halfSize.x ? -pCB->halfSize.x : relSphereCenter.x,
                     relSphereCenter.y > pCB->halfSize.y ? pCB->halfSize.y : 
                       relSphereCenter.y < -pCB->halfSize.y ? -pCB->halfSize.y : relSphereCenter.y,
                     relSphereCenter.z > pCB->halfSize.z ? pCB->halfSize.z : 
                       relSphereCenter.z < -pCB->halfSize.z ? -pCB->halfSize.z : relSphereCenter.z};
  double dist = vLength2(vSub(closestP, relSphereCenter));
  Vector closestPWorld = m34vMult(pCB->pRB->transformMatrix, closestP);
  collisions[collisionC].p = closestPWorld;
  collisions[collisionC].normal = vNorm(vSub(closestPWorld, pCS->pRB->p));
  collisions[collisionC].penetration = pCS->r - fabs(dist);
  collisionC++;
}

#endif
