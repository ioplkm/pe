#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

#include "vector.h"

typedef struct {
  double r, i, j, k;
} Quaternion;

Quaternion qNorm(Quaternion q) {
  double d = q.r*q.r + q.i*q.i + q.j*q.j + q.k*q.k;
  if (!d) return (Quaternion){1, 0, 0, 0};
  d = 1/sqrt(d);
  return (Quaternion){q.r * d, q.i * d, q.j * d, q.k * d};
}

Quaternion qMult(Quaternion q1, Quaternion q2) {
  return (Quaternion){q1.r*q2.r - q1.i*q2.i - q1.j*q2.j - q1.k*q2.k,
                      q1.r*q2.i + q1.i*q2.r + q1.j*q2.k - q1.k*q2.j,
                      q1.r*q2.j - q1.j*q2.r + q1.k*q2.i + q1.i*q2.k,
                      q1.r*q2.k + q1.k*q2.r - q1.i*q2.j + q1.j*q2.i};
}

Quaternion qRotate(Quaternion q, Vector v) {
  return qMult(q, (Quaternion){0, v.x, v.y, v.z});
}

Quaternion qvAdd(Quaternion q, Vector v) {
  Quaternion t = qMult((Quaternion){0, v.x, v.y, v.z}, q);
  return (Quaternion){q.r + t.r/2.0,
                      q.i + t.i/2.0,
                      q.j + t.j/2.0,
                      q.k + t.k/2.0};
}

#endif
