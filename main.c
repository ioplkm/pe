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
  fbInit();
  Rigidbody rb = {{10, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  //Rigidbody rb2 = {{8, 3, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0}, 1, cubeiit, null34};
  Rigidbody rb2 = {{8, 3, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0.901, 0, 0, 0.434}, 1, cubeiit, null34};
  rb2.o = qNorm(rb2.o);
  rb.transformMatrix = m34FromQV(rb.o, rb.p);
  rb2.transformMatrix = m34FromQV(rb2.o, rb2.p);
  CollisionBox cb = {&rb, {0, 0, 0}, {2, 2, 2}};
  CollisionBox cb2 = {&rb2, {0, 0, 0}, {2, 2, 2}};
  Vector p1 = m34vMult(rb2.transformMatrix, (Vector){2, 2, 0});
  Vector p2 = m34vMult(rb2.transformMatrix, (Vector){2, -2, 0});
  Vector p3 = m34vMult(rb2.transformMatrix, (Vector){-2, 2, 0});
  Vector p4 = m34vMult(rb2.transformMatrix, (Vector){-2, -2, 0});
  Vector p5 = m34vMult(rb.transformMatrix, (Vector){2, 2, 0});
  Vector p6 = m34vMult(rb.transformMatrix, (Vector){2, -2, 0});
  Vector p7 = m34vMult(rb.transformMatrix, (Vector){-2, 2, 0});
  Vector p8 = m34vMult(rb.transformMatrix, (Vector){-2, -2, 0});
  //for (;;) {
    drawV(p1, red);
    drawV(p2, red);
    drawV(p3, red);
    drawV(p4, red);
    drawV(p5, blue);
    drawV(p6, blue);
    drawV(p7, blue);
    drawV(p8, blue);
    usleep((int)(1000000*dTime));
    drawV(p1, black);
    drawV(p2, black);
    drawV(p3, black);
    drawV(p4, black);
    drawV(p5, black);
    drawV(p6, black);
    drawV(p7, black);
    drawV(p8, black);
  //}
  
  BoxBoxCollision(&cb, &cb2);
  for (int i = 0; i < 3; i++) {
    printf("pen: %f ", collisions[i].penetration);
    printV(collisions[i].p);
  }
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
