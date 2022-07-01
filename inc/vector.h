#ifndef VECTOR_H
#define VECTOR_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h> //tmp; for printV

typedef struct {
  double x, y, z;
} Vector;

void printV(Vector v);

Vector vAdd(Vector v1, Vector v2);

Vector vSub(Vector v1, Vector v2);

double scalarProd(Vector v1, Vector v2);

Vector vectorProd(Vector v1, Vector v2);

Vector vMult(Vector v, double d);

double vLength(Vector v);

double vLength2(Vector v);

Vector vNormalize(Vector v);

Vector vNorm(Vector v);

bool vIsZero(Vector v);

bool vIsEqual(Vector v1, Vector v2);

double vDist(Vector v1, Vector v2);

double vDist2(Vector v1, Vector v2);

Vector vInv(Vector v);

#endif
