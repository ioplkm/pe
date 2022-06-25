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

#endif
