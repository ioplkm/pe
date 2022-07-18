#include "../inc/resolution.h"

#include "../inc/narrowcollision.h"

void resolveCollision(Collision *pC) {
  Vector lX = vNorm(pC->normal);
  Vector lY = fabs(lX.x) > fabs(lX.y) ? (Vector){0, 1, 0} : (Vector){1, 0, 0};
  Vector lZ = vNorm(vectorProd(lX, lY)); 
  lY = vNorm(vectorProd(lX, lZ));
}
