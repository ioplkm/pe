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
} Sphere;

typedef struct {
  RigidBody *pRB;
  Matrix34 offset;
  Vector normal;
  double planeOffset;
} Plane;

typedef struct {
  Vector p;
  Vector normal;
  double penetration;
} Collision;

Collision collisions[999];
int collisionC;

void SphereSphereCollision(Sphere *pS1, Sphere *pS2) {
  Vector s1p = pS1->pRB->p;
  Vector s2p = pS2->pRB->p;
  Vector midline = vSub(s1p, s2p);
  double size = vLength(midline);

  collisions[collisionC].p = vAdd(s1p, vMult(midline, 0.5));
  collisions[collisionC].normal = vMult(midline, 1/size);
  collisions[collisionC].penetration = pS1->r + pS2->r - size;
  collisionC++;
}

void SpherePlaneCollision(Sphere *pS, Plane *pP) {
  Vector sp = pS->pRB->p;
}

#endif
