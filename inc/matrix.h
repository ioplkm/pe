#ifndef MATRIX_H
#define MATRIX_H

#include "vector.h"
#include "quaternion.h"

typedef struct {
  double data[9];
} Matrix33;

typedef struct {
  double data[12];
} Matrix34;

Vector m33vMult(Matrix33 m, Vector v);

Vector m34vMult(Matrix34 m, Vector v);

Matrix33 m33m33Mult(Matrix33 m1, Matrix33 m2);

Matrix34 m34m34Mult(Matrix34 m1, Matrix34 m2);

Matrix33 m33Invert(Matrix33 m);

Matrix34 m34Invert(Matrix34 m);

Matrix33 m33Transpose(Matrix33 m);

Matrix33 m33FromQ(Quaternion q);

Matrix34 m34FromQV(Quaternion q, Vector v);

/*Vector localToWorld(Vector local, Matrix34 trans);
  return m34vMult(trans, local);
}*/

Vector worldToLocal(Vector world, Matrix34 trans);

Vector localToWorldDir(Vector local, Matrix34 trans);

Vector worldToLocalDir(Vector local, Matrix34 trans);

#endif
