#include "../inc/point.h"

void updatePoint(Point *pPoint, double dTime) {
  Vector aRes = vAdd(pPoint->a, vMult(pPoint->f, pPoint->inverseMass)); 
  //pPoint->v = vMult(   vAdd(pPoint->v, vMult(aRes, dTime)),   pow(0.999, dTime));
  pPoint->v = vAdd(pPoint->v, vMult(aRes, dTime));
  pPoint->p = vAdd(pPoint->p, vAdd(vMult(pPoint->v, dTime), vMult(aRes, dTime * dTime / 2)));
  pPoint->f = (Vector){0, 0, 0};
}
