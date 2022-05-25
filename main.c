#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include "include/spring.h"
#include "include/cable.h"

#include "include/matrix.h"
#include "include/rigidbody.h"
#include "include/rigidbodyspring.h"

#include "include/fb.h"

#define dTime 1/64.0
#define G 9.81

void printCollision(PointCollision *pC) {
  printf("coord1: %7.2f, %7.2f, %7.2f\n", pC->pPoint1->p.x, pC->pPoint1->p.y, pC->pPoint1->p.z);
  printf("coord2: %7.2f, %7.2f, %7.2f\n", pC->pPoint2->p.x, pC->pPoint2->p.y, pC->pPoint2->p.z);
  printf("normal: %7.2f, %7.2f, %7.2f\n", pC->normal.x, pC->normal.y, pC->normal.z);
  printf("penetration: %7.2f\n", pC->penetration);
}

int main() {

  fbInit();

  //pointCollisions = (PointCollision*)malloc(sizeof(PointCollision) * 99);

  Matrix33 iit = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  Matrix34 null34 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  Rigidbody rb = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 1}, 1, iit, null34};
  Rigidbody rb2 = {{10, 10, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 1}, 0, iit, null34};
  RigidbodySpring s = {{0, 10, 0}, {1, 1, 0}, &rb, &rb2, 1, 10};
  for (int i = 0; i < 64*10000; i++) {
    rb.transformMatrix = calcTransformMatrix(rb.p, rb.o);
    rb2.transformMatrix = calcTransformMatrix(rb2.p, rb2.o);
    updateRigidbodySpringForces(&s);
    updateRigidbody(&rb, dTime);
    updateRigidbody(&rb2, dTime);
    drawPoint((int)(rb.p.x * 10) + 960, (int)(-rb.p.y * 10) + 540, red);
    drawPoint((int)(rb2.p.x * 10) + 960, (int)(-rb2.p.y * 10) + 540, yellow);
    usleep((int)(1000000*dTime));
    drawPoint((int)(rb.p.x * 10) + 960, (int)(-rb.p.y * 10) + 540, black);
    drawPoint((int)(rb2.p.x * 10) + 960, (int)(-rb2.p.y * 10) + 540, black);
  }
}
