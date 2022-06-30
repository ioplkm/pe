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

//////////////////////////////////////////////

Vector getTransformRow(Matrix34 m, int n) {
  return (Vector){m.data[n], m.data[n+4], m.data[n+8]};
}

double projectToAxis(CollisionBox *pCB, Vector axis) {
  return pCB->halfSize.x * fabs(scalarProd(axis, getTransformRow(pCB->pRB->transformMatrix, 0))) + 
         pCB->halfSize.y * fabs(scalarProd(axis, getTransformRow(pCB->pRB->transformMatrix, 1))) +
         pCB->halfSize.z * fabs(scalarProd(axis, getTransformRow(pCB->pRB->transformMatrix, 2)));
}

double axisPenetration(CollisionBox *pB1, CollisionBox *pB2, Vector axis) {
  Vector b1ToB2 = vSub(pB2->pRB->p, pB1->pRB->p);
  double project1 = projectToAxis(pB1, axis);
  double project2 = projectToAxis(pB2, axis);
  double dist = fabs(scalarProd(axis, b1ToB2));
  return project1 + project2 - dist;
}

void BoxBoxCollision(CollisionBox *pB1, CollisionBox *pB2) {
  Vector axis[15]; //to optimize later
  Vector b1x = getTransformRow(pB1->pRB->transformMatrix, 0);
  Vector b1y = getTransformRow(pB1->pRB->transformMatrix, 1);
  Vector b1z = getTransformRow(pB1->pRB->transformMatrix, 2);
  Vector b2x = getTransformRow(pB2->pRB->transformMatrix, 0);
  Vector b2y = getTransformRow(pB2->pRB->transformMatrix, 1);
  Vector b2z = getTransformRow(pB2->pRB->transformMatrix, 2);
  axis[0] = b1x;
  axis[1] = b1y;
  axis[2] = b1z;
  axis[3] = b2x;
  axis[4] = b2y;
  axis[5] = b2z;
  axis[6] = vectorProd(b1x, b2x);
  axis[7] = vectorProd(b1x, b2y);
  axis[8] = vectorProd(b1x, b2z);
  axis[9] = vectorProd(b1y, b2x);
  axis[10] = vectorProd(b1y, b2y);
  axis[11] = vectorProd(b1y, b2z);
  axis[12] = vectorProd(b1z, b2x);
  axis[13] = vectorProd(b1z, b2y);
  axis[14] = vectorProd(b1z, b2z);

  double minPen = DBL_MAX;
  for (int i = 0; i < 15; i++) {
    if (vIsZero(axis[i])) continue;
    axis[i] = vNorm(axis[i]);
    double pen = axisPenetration(pB1, pB2, axis[i]);
    printf("%f\n", pen);
    if (pen < minPen) minPen = pen;
  }
  printf("%f\n", minPen);
}

#endif
