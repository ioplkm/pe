#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include "inc/pointspring.h"
#include "inc/pointcable.h"

#include "inc/matrix.h"
#include "inc/rigidbody.h"
#include "inc/spring.h"

#include "inc/broadcollision.h"
#include "inc/narrowcollision.h"

#include <fb.h>

#define dTime 1/64.0
#define G 9.81

void printCollision(PointCollision *pC) {
  printf("coord1: %7.2f, %7.2f, %7.2f\n", pC->pPoint1->p.x, pC->pPoint1->p.y, pC->pPoint1->p.z);
  printf("coord2: %7.2f, %7.2f, %7.2f\n", pC->pPoint2->p.x, pC->pPoint2->p.y, pC->pPoint2->p.z);
  printf("normal: %7.2f, %7.2f, %7.2f\n", pC->normal.x, pC->normal.y, pC->normal.z);
  printf("penetration: %7.2f\n", pC->penetration);
}

Vector transV(Vector v) {
  return (Vector){(int)(v.x * 10 + 960), (int)(-v.y * 10 + 540)};
}

void drawV(Vector v, uint32_t color) {
   drawPoint((int)(v.x * 10) + 960, (int)(-v.y * 10) + 540, color);
}

void drawL(Vector v1, Vector v2, uint32_t color) {
  //printf("call drawline with %f %f %f %f\n", v1.x, v1.y, v2.x, v2.y);
  drawLine(transV(v1).x, transV(v1).y, transV(v2).x, transV(v2).y, color);
}

void drawCB(CollisionBox *pCB, uint32_t color) {
  Vector p1 = m34vMult(pCB->pRB->transformMatrix, (Vector){pCB->halfSize.x, pCB->halfSize.y, 0});
  Vector p2 = m34vMult(pCB->pRB->transformMatrix, (Vector){pCB->halfSize.x, -pCB->halfSize.y, 0});
  Vector p3 = m34vMult(pCB->pRB->transformMatrix, (Vector){-pCB->halfSize.x, -pCB->halfSize.y, 0});
  Vector p4 = m34vMult(pCB->pRB->transformMatrix, (Vector){-pCB->halfSize.x, pCB->halfSize.y, 0});
  drawL(p1, p2, color);
  drawL(p2, p3, color);
  drawL(p3, p4, color);
  drawL(p4, p1, color);
}

void clearCB(CollisionBox *pCB) {
  Vector p1 = m34vMult(pCB->pRB->transformMatrix, (Vector){pCB->halfSize.x, pCB->halfSize.y, 0});
  Vector p2 = m34vMult(pCB->pRB->transformMatrix, (Vector){pCB->halfSize.x, -pCB->halfSize.y, 0});
  Vector p3 = m34vMult(pCB->pRB->transformMatrix, (Vector){-pCB->halfSize.x, -pCB->halfSize.y, 0});
  Vector p4 = m34vMult(pCB->pRB->transformMatrix, (Vector){-pCB->halfSize.x, pCB->halfSize.y, 0});
  drawL(p1, p2, black);
  drawL(p2, p3, black);
  drawL(p3, p4, black);
  drawL(p4, p1, black);
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
  /*Vector p1 = m34vMult(rb2.transformMatrix, (Vector){2, 2, 0});
  Vector p2 = m34vMult(rb2.transformMatrix, (Vector){2, -2, 0});
  Vector p3 = m34vMult(rb2.transformMatrix, (Vector){-2, 2, 0});
  Vector p4 = m34vMult(rb2.transformMatrix, (Vector){-2, -2, 0});
  Vector p5 = m34vMult(rb.transformMatrix, (Vector){2, 2, 0});
  Vector p6 = m34vMult(rb.transformMatrix, (Vector){2, -2, 0});
  Vector p7 = m34vMult(rb.transformMatrix, (Vector){-2, 2, 0});
  Vector p8 = m34vMult(rb.transformMatrix, (Vector){-2, -2, 0});*/
  for (;;) {
    /*//drawV(p1, red);
    //drawV(p2, red);
    drawL(p1, p2, red);
    drawL(p3, p4, red);
    //drawV(p3, red);
    //drawV(p4, red);
    drawV(p5, blue);
    drawV(p6, blue);
    drawV(p7, blue);
    drawV(p8, blue);*/
    drawCB(&cb, red);
    drawCB(&cb2, blue);
    usleep((int)(1000000*dTime));
    clearCB(&cb);
    clearCB(&cb2);
    /*drawV(p1, black);
    drawV(p2, black);
    drawV(p3, black);
    drawV(p4, black);
    drawV(p5, black);
    drawV(p6, black);
    drawV(p7, black);
    drawV(p8, black);*/
  }
  
  BoxBoxCollision(&cb, &cb2);
  /*for (int i = 0; i < 3; i++) {
    printf("pen: %f ", collisions[i].penetration);
    printV(collisions[i].p);
  }*/
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
