#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include "include/spring.h"
#include "include/cable.h"
#include "include/matrix.h"
#include "include/rigidbody.h"

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

  pointCollisions = (PointCollision*)malloc(sizeof(PointCollision) * 99);

  Point p1 = {{7, 7, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, 1};
  Point p2 = {{7, -7, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, 1};
  Point p3 = {{-7, -7, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, 1};
  Point p4 = {{-7, 7, 0}, {0, 0, 0}, {0, -G, 0}, {0, 0, 0}, 1};
  //Point c = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};

  //Cable c1 = {&p1, &c, 10, 0.1};
  //Cable c2 = {&p2, &c, 10, 0.1};
  //Cable c3 = {&p3, &c, 10, 0.1};
  Cable c1c2 = {&p1, &p2, 14, 0.1};
  Cable c2c3 = {&p2, &p3, 14, 0.1};
  Cable c3c4 = {&p3, &p4, 14, 0.1};
  Cable c4c1 = {&p4, &p1, 14, 0.1};
  Cable c1c3 = {&p1, &p3, 14*sqrt(2), 0.1};
  Cable c2c4 = {&p2, &p4, 14*sqrt(2), 0.1};
  //Spring s = {&p1, &p4, 5, 1};
  //Cable rod = {&p3, &p4, 17, 0};
  //AnchoredCable ac = {&p1, {0, 10, 0}, 10, 1};
  for (int i = 0; i < 64*10000; i++) {
    
    //updateSpringForces(s);

    updatePoint(&p1, dTime);
    updatePoint(&p2, dTime);
    updatePoint(&p3, dTime);
    updatePoint(&p4, dTime);
    //updatePoint(&c, dTime);

    collisionsAmount = 0;
    /*createCollisionFromRod(&c1);
    createCollisionFromRod(&c2);
    createCollisionFromRod(&c3);
    createCollisionFromRod(&c4);*/
    createCollisionFromRod(&c2c4);
    createCollisionFromRod(&c1c2);
    createCollisionFromRod(&c2c3);
    createCollisionFromRod(&c3c4);
    createCollisionFromRod(&c4c1);
    createCollisionFromRod(&c1c3);
    //createCollisionFromRod(&c3c4);
    //createCollisionFromRod(&c4c1);
    //createCollisionFromRod(&rod);
    resolveCollisions(dTime);

    drawPoint((int)(p1.p.x * 10) + 960, (int)(-p1.p.y * 10) + 540, red);
    drawPoint((int)(p2.p.x * 10) + 960, (int)(-p2.p.y * 10) + 540, yellow);
    drawPoint((int)(p3.p.x * 10) + 960, (int)(-p3.p.y * 10) + 540, green);
    drawPoint((int)(p4.p.x * 10) + 960, (int)(-p4.p.y * 10) + 540, blue);
    //drawPoint((int)(c.p.x * 10) + 960, (int)(-c.p.y * 10) + 540, white);
    //drawPoint((int)(0 * 50) + 960, (int)(0 * 50) + 540, green);
    usleep((int)(1000000*dTime));
    drawPoint((int)(p1.p.x * 10) + 960, (int)(-p1.p.y * 10) + 540, black);
    drawPoint((int)(p2.p.x * 10) + 960, (int)(-p2.p.y * 10) + 540, black);
    drawPoint((int)(p3.p.x * 10) + 960, (int)(-p3.p.y * 10) + 540, black);
    drawPoint((int)(p4.p.x * 10) + 960, (int)(-p4.p.y * 10) + 540, black);
    //drawPoint((int)(c.p.x * 10) + 960, (int)(-c.p.y * 10) + 540, black);
    //drawPoint((int)(0 * 50) + 960, (int)(0 * 50) + 540, black);
  }
}
