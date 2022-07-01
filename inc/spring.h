#ifndef SPRING_H
#define SPRING_H

#include <math.h>

#include "rigidbody.h"
#include "matrix.h"

typedef struct {
  Vector p1, p2;
  Rigidbody *pRB1, *pRB2;
  double k, l;
} RigidbodySpring;

void updateRigidbodySpringForces(RigidbodySpring *pS);

#endif
