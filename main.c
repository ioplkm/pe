#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include "include/pointspring.h"
#include "include/pointcable.h"

#include "include/matrix.h"
#include "include/rigidbody.h"
#include "include/spring.h"

#include "include/broadcollision.h"
#include "include/narrowcollision.h"

#include "include/fb.h"

#define dTime 1/64.0
#define G 9.81

void printCollision(PointCollision *pC) {
  printf("coord1: %7.2f, %7.2f, %7.2f\n", pC->pPoint1->p.x, pC->pPoint1->p.y, pC->pPoint1->p.z);
  printf("coord2: %7.2f, %7.2f, %7.2f\n", pC->pPoint2->p.x, pC->pPoint2->p.y, pC->pPoint2->p.z);
  printf("normal: %7.2f, %7.2f, %7.2f\n", pC->normal.x, pC->normal.y, pC->normal.z);
  printf("penetration: %7.2f\n", pC->penetration);
}

Matrix33 cubeiit = {1, 0, 0, 0, 1, 0, 0, 0, 1};
Matrix34 null34 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int main() {
  Rigidbody rb = {{10, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  Rigidbody rb2 = {{8, 0, 3}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  rb.transformMatrix = m34FromQV(rb.o, rb.p);
  rb2.transformMatrix = m34FromQV(rb2.o, rb2.p);
  CollisionBox cb = {&rb, {0, 0, 0}, {2, 2, 2}};
  CollisionBox cb2 = {&rb2, {0, 0, 0}, {2, 2, 2}};
  //Vector axis = vNorm((Vector) {1, 1, 0});
  //double pen = axisPenetration(&cb, &cb2, axis);
  //double projection = projectToAxis(&cb, axis);
  BoxBoxCollision(&cb, &cb2);
  //printf("%f\n", pen);
}

/*int main() {

  fbInit();

  //pointCollisions = (PointCollision*)malloc(sizeof(PointCollision) * 99);

  Matrix33 cubeiit = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  Matrix34 null34 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  Rigidbody rb = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  Rigidbody rb2 = {{10, 10, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 0, cubeiit, null34};
  RigidbodySpring s = {{2, 2, 0}, {2, 2, 0}, &rb, &rb2, 1, 10};
  for (int i = 0; i < 64*10000; i++) {
    rb.transformMatrix = m34FromQV(rb.o, rb.p);
    rb2.transformMatrix = m34FromQV(rb2.o, rb2.p);
    updateRigidbodySpringForces(&s);
    updateRigidbody(&rb, dTime);
    updateRigidbody(&rb2, dTime);

    Vector rbp = m34vMult(rb.transformMatrix, s.p1);
    Vector rbp2 = m34vMult(rb2.transformMatrix, s.p2);
    drawV(rb.p, red);
    drawV(rb2.p, yellow);
    drawV(rbp, green);
    drawV(rbp2, blue);
    usleep((int)(1000000*dTime));
    drawV(rb.p, black);
    drawV(rb2.p, black);
    drawV(rbp, black);
    drawV(rbp2, black);
  }
}*/
