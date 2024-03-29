#include "../inc/vector.h"

void printV(Vector v) {
  printf("x:%7.2f y:%7.2f z:%7.2f\n", v.x, v.y, v.z);
}

Vector vAdd(Vector v1, Vector v2) {
  return (Vector){v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

Vector vSub(Vector v1, Vector v2) {
  return (Vector){v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

double scalarProd(Vector v1, Vector v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector vectorProd(Vector v1, Vector v2) {
  return (Vector){v1.y * v2.z - v1.z * v2.y,
                  v1.z * v2.x - v1.x * v2.z,
                  v1.x * v2.y - v1.y * v2.x};
}

Vector vMult(Vector v, double d) {
  return (Vector){v.x * d, v.y * d, v.z * d};
}

Vector vDiv(Vector v, double d) {
  return (Vector){v.x / d, v.y / d, v.z / d};
}

double vLength(Vector v) {
  return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

double vLength2(Vector v) {
  return v.x*v.x + v.y*v.y + v.z*v.z;
}

Vector vNormalize(Vector v) {
  return vMult(v, 1 / vLength(v));
}

Vector vNorm(Vector v) {
  return vMult(v, 1 / vLength(v));
}

bool vIsZero(Vector v) {
  return !v.x && !v.y && !v.z;
}

bool vIsEqual(Vector v1, Vector v2) {
  return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

double vDist(Vector v1, Vector v2) {
  return vLength(vSub(v1, v2));
} 

double vDist2(Vector v1, Vector v2) {
  return vLength2(vSub(v1, v2));
} 

Vector vInv(Vector v) {
  return (Vector){-v.x, -v.y, -v.z};
}
