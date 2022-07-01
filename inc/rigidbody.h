#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "quaternion.h"
#include "matrix.h"

typedef struct {
  Vector p, v, r, a, f, t;
  Quaternion o;
  double inverseMass;
  Matrix33 inverseInertiaTensor;

  Matrix34 transformMatrix;
} Rigidbody;

/*Matrix34 calcTransformMatrix(Vector p, Quaternion o) {
  return m34FromQV(o, p);
}*/

Matrix33 calcInverseInertiaTensorWorld(Matrix33 iit, Matrix34 transform);

void applyForceAtPoint(Rigidbody *pRB, Vector f, Vector p);

void updateRigidbody(Rigidbody *pRB, double dTime);

#endif
