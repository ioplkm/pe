#ifndef NARROWCOLLISION_H
#define NARROWCOLLISION_H

#include <float.h> //for DBL_MAX

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
  double r;
  Rigidbody *pB1, *pB2;
} Collision;

//Collision collisions[999];
//int collisionC;

/*void SphereSphereCollision(CollisionSphere *pCS1, CollisionSphere *pCS2) {
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
  Vector axes[15]; //to optimize later
  Vector b1x = getTransformRow(pB1->pRB->transformMatrix, 0);
  Vector b1y = getTransformRow(pB1->pRB->transformMatrix, 1);
  Vector b1z = getTransformRow(pB1->pRB->transformMatrix, 2);
  Vector b2x = getTransformRow(pB2->pRB->transformMatrix, 0);
  Vector b2y = getTransformRow(pB2->pRB->transformMatrix, 1);
  Vector b2z = getTransformRow(pB2->pRB->transformMatrix, 2);
  axes[0] = b1x;
  axes[1] = b1y;
  axes[2] = b1z;
  axes[3] = b2x;
  axes[4] = b2y;
  axes[5] = b2z;
  axes[6] = vectorProd(b1x, b2x);
  axes[7] = vectorProd(b1x, b2y);
  axes[8] = vectorProd(b1x, b2z);
  axes[9] = vectorProd(b1y, b2x);
  axes[10] = vectorProd(b1y, b2y);
  axes[11] = vectorProd(b1y, b2z);
  axes[12] = vectorProd(b1z, b2x);
  axes[13] = vectorProd(b1z, b2y);
  axes[14] = vectorProd(b1z, b2z);

  Vector axis;
  double minPen = DBL_MAX;
  int best;
  for (int i = 0; i < 15; i++) {
    axis = axes[i];
    //if (vIsZero(axis)) continue;
    if (vIsZero(axis)) {printf("drop\n"); continue;}
    axis = vNorm(axis);
    double pen = axisPenetration(pB1, pB2, axis);
    printf("pen: %f\n", pen);
    if (pen <= minPen) {
      minPen = pen;
      best = i;
    }
  }

  printf("best: %d\n", best);

  if (best < 3) {
    Vector normal = getTransformRow(pB1->pRB->transformMatrix, best);
    if (scalarProd(normal, vSub(pB2->pRB->p, pB1->pRB->p)) > 0) normal = vInv(normal);
    Vector vertex = pB2->halfSize;
    if (scalarProd(getTransformRow(pB2->pRB->transformMatrix, 0), normal) < 0) vertex.x *= -1;
    if (scalarProd(getTransformRow(pB2->pRB->transformMatrix, 1), normal) < 0) vertex.y *= -1;
    if (scalarProd(getTransformRow(pB2->pRB->transformMatrix, 2), normal) < 0) vertex.z *= -1;

    collisions[collisionC].p = m34vMult(pB2->pRB->transformMatrix, vertex);
    collisions[collisionC].normal = normal;
    collisions[collisionC].penetration = minPen;
  } else if (best < 6) {
    best -= 3;
    Vector normal = getTransformRow(pB2->pRB->transformMatrix, best);
    if (scalarProd(normal, vSub(pB1->pRB->p, pB2->pRB->p)) > 0) normal = vInv(normal);
    Vector vertex = pB1->halfSize;
    if (scalarProd(getTransformRow(pB1->pRB->transformMatrix, 0), normal) < 0) vertex.x *= -1;
    if (scalarProd(getTransformRow(pB1->pRB->transformMatrix, 1), normal) < 0) vertex.y *= -1;
    if (scalarProd(getTransformRow(pB1->pRB->transformMatrix, 2), normal) < 0) vertex.z *= -1;

    collisions[collisionC].p = m34vMult(pB1->pRB->transformMatrix, vertex);
    collisions[collisionC].normal = normal;
    collisions[collisionC].penetration = minPen;
  } else {
    best -= 6;
    int oneAxisIndex = best / 3;
    int twoAxisIndex = best % 3;
    Vector oneAxis = getTransformRow(pB1->pRB->transformMatrix, oneAxisIndex);
    Vector twoAxis = getTransformRow(pB2->pRB->transformMatrix, twoAxisIndex);
    Vector axis = vectorProd(oneAxis, twoAxis);
    axis = vNorm(axis);
    if (scalarProd(axis, vSub(pB2->pRB->p, pB1->pRB->p)) > 0) axis = vInv(axis);

    Vector ptOnEdge1 = pB1->halfSize;
    if (oneAxisIndex == 0) ptOnEdge1.x = 0;
    else if (scalarProd(getTransformRow(pB1->pRB->transformMatrix, 0), axis) > 0) ptOnEdge1.x = -ptOnEdge1.x;
    if (oneAxisIndex == 1) ptOnEdge1.y = 0;
    else if (scalarProd(getTransformRow(pB1->pRB->transformMatrix, 1), axis) > 0) ptOnEdge1.y = -ptOnEdge1.y;
    if (oneAxisIndex == 2) ptOnEdge1.z = 0;
    else if (scalarProd(getTransformRow(pB1->pRB->transformMatrix, 2), axis) > 0) ptOnEdge1.z = -ptOnEdge1.z;
    ptOnEdge1 = m34vMult(pB1->pRB->transformMatrix, ptOnEdge1);

    Vector ptOnEdge2 = pB2->halfSize;
    if (twoAxisIndex == 0) ptOnEdge2.x = 0;
    else if (scalarProd(getTransformRow(pB2->pRB->transformMatrix, 0), axis) < 0) ptOnEdge2.x = -ptOnEdge2.x;
    if (twoAxisIndex == 1) ptOnEdge2.y = 0;
    else if (scalarProd(getTransformRow(pB2->pRB->transformMatrix, 1), axis) < 0) ptOnEdge2.y = -ptOnEdge2.y;
    if (twoAxisIndex == 2) ptOnEdge2.z = 0;
    else if (scalarProd(getTransformRow(pB2->pRB->transformMatrix, 2), axis) < 0) ptOnEdge2.z = -ptOnEdge2.z;
    ptOnEdge2 = m34vMult(pB2->pRB->transformMatrix, ptOnEdge2);

    Vector p2p1 = vSub(ptOnEdge1, ptOnEdge2);
    double ds1 = scalarProd(oneAxis, p2p1);
    double ds2 = scalarProd(twoAxis, p2p1);
    double sl1 = vLength2(oneAxis);
    double sl2 = vLength2(twoAxis);
    double sp = scalarProd(oneAxis, twoAxis);
    double denom = sl1 * sl2 - sp*sp;
    double a = (sp * ds2 - sl2 * ds1) / denom;
    double b = (sl1 * ds2 - sp * ds1) / denom;
    Vector p = vAdd(vMult(vAdd(ptOnEdge1, vMult(oneAxis, a)), 0.5),
                    vMult(vAdd(ptOnEdge2, vMult(twoAxis, b)), 0.5));
    collisions[collisionC].p = p;
    collisions[collisionC].normal = axis;
    collisions[collisionC].penetration = minPen;
  }
}
*/
int BoxBoxCollision(CollisionBox *pB1, CollisionBox *pB2, Collision *pC);
#endif
