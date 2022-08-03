#include "../inc/collision.h"

double penetrationOnAxis(ConvexPolyhedra *pP1, ConvexPolyhedra *pP2, Vector axis) {
  Vector p1[8];//convex polyhedra vertices in world coordinates
  Vector p2[8];//convex polyhedra vertices in world coordinates
  for (int i = 0; i < 8; i++) {
    p1[i].x = i & 4 ? pP1->halfSize.x : -pP1->halfSize.x;
    p1[i].y = i & 2 ? pP1->halfSize.y : -pP1->halfSize.y;
    p1[i].z = i & 1 ? pP1->halfSize.z : -pP1->halfSize.z;
    p1[i] = m34vMult(pP1->pRB->transform, p1[i]);
    p2[i].x = i & 4 ? pP2->halfSize.x : -pP2->halfSize.x;
    p2[i].y = i & 2 ? pP2->halfSize.y : -pP2->halfSize.y;
    p2[i].z = i & 1 ? pP2->halfSize.z : -pP2->halfSize.z;
    p2[i] = m34vMult(pP2->pRB->transform, p2[i]);
  }

  double min1 = DBL_MAX;
  double max1 = -DBL_MAX;
  double min2 = DBL_MAX;
  double max2 = -DBL_MAX;
  for (int i = 0; i < 8; i++) {
    double n = scalarProd(axis, p1[i]);
    if (n < min1) min1 = n;
    if (n > max1) max1 = n;
    n = scalarProd(axis, p2[i]);
    if (n < min2) min2 = n;
    if (n > max2) max2 = n;
  }
  return ((max1 < max2 ? max1 : max2) - (min1 > min2 ? min1 : min2)); 
}

/*int minVerticesOnAxis(ConvexPolyhedra *pP, Vector axis, Vector *pVertices) {
  Vector p[8];//convex polyhedra vertices in world coordinates
  for (int i = 0; i < 8; i++) {
    p[i].x = i & 4 ? pP->halfSize.x : -pP->halfSize.x;
    p[i].y = i & 2 ? pP->halfSize.y : -pP->halfSize.y;
    p[i].z = i & 1 ? pP->halfSize.z : -pP->halfSize.z;
    p[i] = m34vMult(pP->pRB->transform, p[i]);
  }
  //finding minimal penetration
  double min = DBL_MAX;
  for (int i = 0; i < 8; i++) {
    double n = scalarProd(axis, p[i]);
    if (n < min) {
      min = n;
    }
  }
  //finding all vetrices with minimal penetration
  int c = 0;
  for (int i = 0; i < 8; i++) {
    double n = scalarProd(axis, p[i]);
    if (n == min) {
      pVertices[c++] = p[i];
    }
  }
  return c--;
}*/

int minVerticesOnAxis(ConvexPolyhedra *pP, Vector axis, Vector center, Vector *pVertices) {
  Vector p[8];//convex polyhedra vertices in world coordinates
  for (int i = 0; i < 8; i++) {
    p[i].x = i & 4 ? pP->halfSize.x : -pP->halfSize.x;
    p[i].y = i & 2 ? pP->halfSize.y : -pP->halfSize.y;
    p[i].z = i & 1 ? pP->halfSize.z : -pP->halfSize.z;
    p[i] = m34vMult(pP->pRB->transform, p[i]);
  }
  //finding minimal penetration
  double min = DBL_MAX;
  for (int i = 0; i < 8; i++) {
    double n = scalarProd(axis, p[i]);
    if (n < min) {
      min = n;
    }
  }
  //finding all vetrices with minimal penetration
  int c = 0;
  Vector mpv[99];
  for (int i = 0; i < 8; i++) {
    double n = scalarProd(axis, p[i]);
    if (n == min) {
      mpv[c++] = p[i];
    }
  }
  //finding minimal distance between each vertex and center
  double minDist = DBL_MAX;
  for (int i = 0; i < c; i++) {
    double dist = vDist(mpv[i], center);
    if (dist < minDist) minDist = dist;
  }
  int C = 0;
  for (int i = 0; i < c; i++) {
    double dist = vDist(mpv[i], center);
    if (dist == minDist) {
      pVertices[C++] = mpv[i];
    };
  }
  return C--;
}

int collision(ConvexPolyhedra *pP1, ConvexPolyhedra *pP2, Collision *pC) {
  Vector norm1[6]; //auxiliary axes
  Vector norm2[6]; //auxiliary axes
  Vector edge1[12]; //auxiliary axes
  Vector edge2[12]; //auxiliary axes
  for (int i = 0; i < 3; i++) {
    norm1[i] = (Vector){pP1->pRB->transform.data[0+i], pP1->pRB->transform.data[4+i], pP1->pRB->transform.data[8+i]};
    for (int j = 0; j < 4; j++) edge1[i*4+j] = norm1[i];
    norm2[i] = (Vector){pP2->pRB->transform.data[0+i], pP2->pRB->transform.data[4+i], pP2->pRB->transform.data[8+i]};
    for (int j = 0; j < 4; j++) edge2[i*4+j] = norm2[i];
  }
  for (int i = 3; i < 6; i++) {
    norm1[i] = vInv(norm1[i-3]);
    norm2[i] = vInv(norm2[i-3]);
  }
  Vector axes[156]; //axes to check in SAT
  for (int i = 0; i < 6; i++) axes[i] = norm1[i];
  for (int i = 6; i < 12; i++) axes[i] = norm2[i-6];
  for (int i = 12; i < 156; i++) axes[i] = vectorProd(edge1[(i-12)/12], edge2[(i-12)%12]);
  //finding minimal interpenetration
  double minPen = DBL_MAX;
  double pen;
  for (int i = 0; i < 156; i++) {
    if (vIsZero(axes[i])) continue;
    pen = penetrationOnAxis(pP1, pP2, vNorm(axes[i]));
    if (pen < minPen) minPen = pen;
  }
  //creating contacts
  int contacts = 0;
  Vector p1p2 = vSub(pP2->pRB->p, pP1->pRB->p);
  for (int i = 0; i < 6; i++) {
    pen = penetrationOnAxis(pP1, pP2, vNorm(axes[i]));
    if (pen != minPen || scalarProd(axes[i], p1p2) <= 0) continue;
    Vector vertices[4];
    int contactsOnAxis = minVerticesOnAxis(pP2, axes[i], pP1->pRB->p, vertices);
    for (int j = 0; j < contactsOnAxis; j++) {
      pC->p = vertices[j];
      pC->normal = axes[i];
      pC->penetration = pen;
      pC->r = 0;
      pC->pB1 = pP1->pRB;
      pC++->pB2 = pP2->pRB;
      contacts++;
    }
  }
  Vector p2p1 = vInv(p1p2);
  for (int i = 6; i < 12; i++) {
    pen = penetrationOnAxis(pP1, pP2, vNorm(axes[i]));
    if (pen != minPen || scalarProd(axes[i], p2p1) <= 0) continue;
    Vector vertices[4];
    int contactsOnAxis = minVerticesOnAxis(pP1, axes[i], pP2->pRB->p, vertices);
    for (int j = 0; j < contactsOnAxis; j++) {
      pC->p = vertices[j];
      pC->normal = axes[i];
      pC->penetration = pen;
      pC->r = 0;
      pC->pB1 = pP1->pRB;
      pC++->pB2 = pP2->pRB;
      contacts++;
    }
  }
  return contacts;
}
