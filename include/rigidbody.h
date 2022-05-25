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

Matrix34 calcTransformMatrix(Vector p, Quaternion o) {
  return m34FromQV(o, p);
}

Matrix33 calcInverseInertiaTensorWorld(Matrix33 iit, Matrix34 transform) {
  double t4 = transform.data[0]*iit.data[0] + transform.data[1]*iit.data[3] + transform.data[2]*iit.data[6];
  double t9 = transform.data[0]*iit.data[1] + transform.data[1]*iit.data[4] + transform.data[2]*iit.data[7];
  double t14 = transform.data[0]*iit.data[2] + transform.data[1]*iit.data[5] + transform.data[2]*iit.data[8];
  double t28 = transform.data[4]*iit.data[0] + transform.data[5]*iit.data[3] + transform.data[6]*iit.data[6];
  double t33 = transform.data[4]*iit.data[1] + transform.data[5]*iit.data[4] + transform.data[6]*iit.data[7];
  double t38 = transform.data[4]*iit.data[2] + transform.data[5]*iit.data[5] + transform.data[6]*iit.data[8];
  double t52 = transform.data[8]*iit.data[0] + transform.data[9]*iit.data[3] + transform.data[10]*iit.data[6];
  double t57 = transform.data[8]*iit.data[1] + transform.data[9]*iit.data[4] + transform.data[10]*iit.data[7];
  double t62 = transform.data[8]*iit.data[2] + transform.data[9]*iit.data[5] + transform.data[10]*iit.data[8];
  return (Matrix33){t4*transform.data[0] + t9*transform.data[1] + t14*transform.data[2],
                    t4*transform.data[4] + t9*transform.data[5] + t14*transform.data[6],
                    t4*transform.data[8] + t9*transform.data[9] + t14*transform.data[10],
                    t28*transform.data[0] + t33*transform.data[1] + t38*transform.data[2],
                    t28*transform.data[4] + t33*transform.data[5] + t38*transform.data[6],
                    t28*transform.data[8] + t33*transform.data[9] + t38*transform.data[10],
                    t52*transform.data[0] + t57*transform.data[1] + t62*transform.data[2],
                    t52*transform.data[4] + t57*transform.data[5] + t62*transform.data[6],
                    t52*transform.data[8] + t57*transform.data[9] + t62*transform.data[10]};
}

void updateRigidbody(Rigidbody *pRB, double dTime) {
  pRB->f = (Vector){0, 0, 0};
  pRB->t = (Vector){0, 0, 0};
}

void applyForceAtPoint(Rigidbody *pRB, Vector f, Vector p) {
  pRB->f = vAdd(pRB->f, f);
  pRB->t = vAdd(pRB->t, vectorProd(vSub(p, pRB->p), f));
}

#endif
