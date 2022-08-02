#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

#include "vector.h"

typedef struct {
  double r, i, j, k;
} Quaternion;

Quaternion qNorm(Quaternion q);

Quaternion qMult(Quaternion q1, Quaternion q2);

Quaternion qRotate(Quaternion q, Vector v);

Quaternion qvAdd(Quaternion q, Vector v);
 
#endif
