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

int minVerticesOnAxis(ConvexPolyhedra *pP, Vector axis, Vector *sum) {
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
    if (n < min) min = n;
  }
  //finding all vetrices with minimal penetration
  int c = 0;
  Vector sumP = (Vector){0, 0, 0};
  for (int i = 0; i < 8; i++) {
    if (scalarProd(axis, p[i]) == min) {
      sumP = vAdd(sumP, p[i]);
      c++;
    }
  }
  sum->x = sumP.x;
  sum->y = sumP.y;
  sum->z = sumP.z;
  return c;
}

int collision(ConvexPolyhedra *pP1, ConvexPolyhedra *pP2, Collision *pC) {
  Vector norm1[6]; //normals of p1
  Vector norm2[6]; //normals of p2
  Vector edge1[12]; //edges of p1
  Vector edge2[12]; //edges of p1
  for (int i = 0; i < 3; i++) {
    norm1[i] = (Vector){pP1->pRB->transform.data[0+i], pP1->pRB->transform.data[4+i], pP1->pRB->transform.data[8+i]};
    for (int j = 0; j < 4; j++) edge1[i*4+j] = vMult(norm1[i], 4);
    norm2[i] = (Vector){pP2->pRB->transform.data[0+i], pP2->pRB->transform.data[4+i], pP2->pRB->transform.data[8+i]};
    for (int j = 0; j < 4; j++) edge2[i*4+j] = vMult(norm2[i], 4);
  }
  for (int i = 3; i < 6; i++) {
    norm1[i] = vInv(norm1[i-3]);
    norm2[i] = vInv(norm2[i-3]);
  }
  //finding axes to check in SAT
  Vector axes[156];
  for (int i = 0; i < 6; i++) axes[i] = norm1[i];
  for (int i = 6; i < 12; i++) axes[i] = norm2[i-6];
  for (int i = 12; i < 156; i++) axes[i] = vectorProd(edge1[(i-12)/12], edge2[(i-12)%12]);
  //finding axes with min penetration
  double pens[156];
  double minPen = DBL_MAX;
  bool isMinPen[156];
  for (int i = 0; i < 156; i++) {
    if (vIsZero(axes[i])) {pens[i] = DBL_MAX; continue;}
    pens[i] = penetrationOnAxis(pP1, pP2, vNorm(axes[i]));
    if (pens[i] < minPen) minPen = pens[i];
  }
  if (minPen < 0) return 0;
  for (int i = 0; i < 156; i++) isMinPen[i] = pens[i] == minPen;
  //creating contacts
  Vector sumP = (Vector){0, 0, 0};
  Vector normal;
  int contacts = 0;
  Vector p1p2 = vSub(pP2->pRB->p, pP1->pRB->p);
  for (int i = 0; i < 6; i++) {
    if (!isMinPen[i]) continue;
    if (scalarProd(axes[i], p1p2) <= 0) continue;
    normal = axes[i];
    Vector toSumP;
    contacts += minVerticesOnAxis(pP2, axes[i], &toSumP);
    sumP = vAdd(sumP, toSumP);
  }
  for (int i = 6; i < 12; i++) {
    if (!isMinPen[i]) continue; 
    if (scalarProd(axes[i], p1p2) >= 0) continue;
    normal = axes[i];
    Vector toSumP;
    contacts += minVerticesOnAxis(pP1, axes[i], &toSumP);
    sumP = vAdd(sumP, toSumP);
  }
  Vector edgeP1[12];
  Vector edgeP2[12];
  for (int i = 0; i < 4; i++) {
    edgeP1[i].x = 0;
    edgeP1[i].y = i & 2 ? -2 : 2;
    edgeP1[i].z = i & 1 ? 2 : -2;
    edgeP2[i] = edgeP1[i];
  }
  for (int i = 4; i < 8; i++) {
    edgeP1[i].y = 0;
    edgeP1[i].x = i & 2 ? -2 : 2;
    edgeP1[i].z = i & 1 ? 2 : -2;
    edgeP2[i] = edgeP1[i];
  }
  for (int i = 8; i < 12; i++) {
    edgeP1[i].z = 0;
    edgeP1[i].x = i & 2 ? -2 : 2;
    edgeP1[i].y = i & 1 ? 2 : -2;
    edgeP2[i] = edgeP1[i];
  }
  double minDist = DBL_MAX;
  double dist;
  Vector p1, p2;
  Vector p1w, p2w;
  Vector p2p1w;
  double edge1p, edge2p;
  Vector e1ps[156];
  Vector e2ps[156];
  for (int i = 12; i < 156; i++) {
    if (!isMinPen[i]) continue;
    Vector e1 = edge1[(i - 12) / 12];
    Vector e2 = edge2[(i - 12) % 12];
    if (vIsZero(vectorProd(e1, e2))) continue;
    p1 = edgeP1[(i - 12) / 12];
    p2 = edgeP2[(i - 12) % 12];
    p1w = m34vMult(pP1->pRB->transform, p1);
    p2w = m34vMult(pP2->pRB->transform, p2);
    p2p1w = vSub(p1w, p2w);
    edge1p = scalarProd(e1, p2p1w);
    edge2p = scalarProd(e2, p2p1w);
    //don't know what is this
    double sm1 = vLength2(e1);
    double sm2 = vLength2(e2);
    double spe = scalarProd(e1, e2);
    double denom = sm1 * sm2 - spe * spe;
    double a = (spe * edge2p - sm2 * edge1p) / denom;
    double b = (sm1 * edge2p - spe * edge1p) / denom;
    e1ps[i] = vAdd(p1w, vMult(e1, a));
    e2ps[i] = vAdd(p2w, vMult(e2, b));
    dist = vDist(e1ps[i], e2ps[i]);
    if (dist < minDist) minDist = dist;
  }
  for (int i = 12; i < 156; i++) {
    if (!isMinPen[i]) continue;
    Vector e1 = edge1[(i - 12) / 12];
    Vector e2 = edge2[(i - 12) % 12];
    if (vIsZero(vectorProd(e1, e2))) continue;
    if (vDist(e1ps[i], e2ps[i]) != minDist) continue;
    Vector p = vAdd(vMult(e1ps[i], 0.5), vMult(e2ps[i], 0.5));
    normal = vNorm(axes[i]);
    sumP = vAdd(sumP, p);
    contacts++;
  }
  if (scalarProd(normal, p1p2) < 0) normal = vInv(normal);
  pC->p = vDiv(sumP, contacts);
  pC->normal = normal;
  pC->penetration = minPen;
  pC->r = 0;
  pC->pB1 = pP1->pRB;
  pC->pB2 = pP2->pRB;
  return 1;
}
