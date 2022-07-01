#include "../inc/matrix.h"

Vector m33vMult(Matrix33 m, Vector v) {
  return (Vector){v.x * m.data[0] + v.y * m.data[1] + v.z * m.data[2],
                  v.x * m.data[3] + v.y * m.data[4] + v.z * m.data[5],
                  v.x * m.data[6] + v.y * m.data[7] + v.z * m.data[8]};
}

Vector m34vMult(Matrix34 m, Vector v) {
  return (Vector){v.x * m.data[0] + v.y * m.data[1] + v.z * m.data[2] + m.data[3],
                  v.x * m.data[4] + v.y * m.data[5] + v.z * m.data[6] + m.data[7],
                  v.x * m.data[8] + v.y * m.data[9] + v.z * m.data[10] + m.data[11]};
}

Matrix33 m33m33Mult(Matrix33 m1, Matrix33 m2) {
  return (Matrix33){m1.data[0]*m2.data[0] + m1.data[1]*m2.data[3] + m1.data[2]*m2.data[6],
                    m1.data[0]*m2.data[1] + m1.data[1]*m2.data[4] + m1.data[2]*m2.data[7],
                    m1.data[0]*m2.data[2] + m1.data[1]*m2.data[5] + m1.data[2]*m2.data[8],

                    m1.data[3]*m2.data[0] + m1.data[4]*m2.data[3] + m1.data[5]*m2.data[6],
                    m1.data[3]*m2.data[1] + m1.data[4]*m2.data[4] + m1.data[5]*m2.data[7],
                    m1.data[3]*m2.data[2] + m1.data[4]*m2.data[5] + m1.data[5]*m2.data[8],
                    
                    m1.data[6]*m2.data[0] + m1.data[7]*m2.data[3] + m1.data[8]*m2.data[6],
                    m1.data[6]*m2.data[1] + m1.data[7]*m2.data[4] + m1.data[8]*m2.data[7],
                    m1.data[6]*m2.data[2] + m1.data[7]*m2.data[5] + m1.data[8]*m2.data[8]};
}

Matrix34 m34m34Mult(Matrix34 m1, Matrix34 m2) {
  return (Matrix34){m2.data[0]*m1.data[0] + m2.data[4]*m1.data[1] + m2.data[8]*m1.data[2],
                    m2.data[1]*m1.data[0] + m2.data[5]*m1.data[1] + m2.data[9]*m1.data[2],
                    m2.data[2]*m1.data[0] + m2.data[6]*m1.data[1] + m2.data[10]*m1.data[2],
                    m2.data[3]*m1.data[0] + m2.data[7]*m1.data[1] + m2.data[11]*m1.data[2],

                    m2.data[0]*m1.data[4] + m2.data[4]*m1.data[5] + m2.data[8]*m1.data[6],
                    m2.data[1]*m1.data[4] + m2.data[5]*m1.data[5] + m2.data[9]*m1.data[6],
                    m2.data[2]*m1.data[4] + m2.data[6]*m1.data[5] + m2.data[10]*m1.data[6],
                    m2.data[3]*m1.data[4] + m2.data[7]*m1.data[5] + m2.data[11]*m1.data[6],

                    m2.data[0]*m1.data[8] + m2.data[4]*m1.data[9] + m2.data[8]*m1.data[10],
                    m2.data[1]*m1.data[8] + m2.data[5]*m1.data[9] + m2.data[9]*m1.data[10],
                    m2.data[2]*m1.data[8] + m2.data[6]*m1.data[9] + m2.data[10]*m1.data[10],
                    m2.data[3]*m1.data[8] + m2.data[7]*m1.data[9] + m2.data[11]*m1.data[10]};
}

Matrix33 m33Invert(Matrix33 m) {
  double determinant = m.data[0] * m.data[4] * m.data[8] +
                       m.data[0] * m.data[5] * m.data[7] +
                       m.data[1] * m.data[3] * m.data[8] +
                       m.data[2] * m.data[3] * m.data[7] +
                       m.data[1] * m.data[6] * m.data[5] +
                       m.data[2] * m.data[6] * m.data[6];
  if (!determinant) return m;
  double invD = 1 / determinant;
  return (Matrix33){(m.data[4]*m.data[8] - m.data[5]*m.data[7]) * invD,
                    (m.data[2]*m.data[7] - m.data[1]*m.data[8]) * invD,
                    (m.data[1]*m.data[5] - m.data[2]*m.data[4]) * invD,

                    (m.data[5]*m.data[6] - m.data[3]*m.data[8]) * invD,
                    (m.data[0]*m.data[8] - m.data[2]*m.data[6]) * invD,
                    (m.data[2]*m.data[3] - m.data[0]*m.data[5]) * invD,

                    (m.data[3]*m.data[7] - m.data[4]*m.data[6]) * invD,
                    (m.data[1]*m.data[6] - m.data[0]*m.data[7]) * invD,
                    (m.data[0]*m.data[4] - m.data[1]*m.data[3]) * invD};
}

Matrix34 m34Invert(Matrix34 m) {
  double determinant = m.data[2]*m.data[5]*m.data[8] +
                       m.data[2]*m.data[4]*m.data[9] +
                       m.data[1]*m.data[6]*m.data[8] -
                       m.data[0]*m.data[6]*m.data[9] -
                       m.data[1]*m.data[4]*m.data[10] +
                       m.data[0]*m.data[5]*m.data[10];
  if (!determinant) return m;
  double invD = 1 / determinant;
  return (Matrix34){(m.data[5]*m.data[10] - m.data[6]*m.data[9]) * invD,
                    (m.data[2]*m.data[9] - m.data[1]*m.data[10]) * invD,
                    (m.data[1]*m.data[6] - m.data[2]*m.data[5]) * invD,
                    (m.data[3]*m.data[6]*m.data[9] -
                     m.data[3]*m.data[5]*m.data[10] -
                     m.data[2]*m.data[7]*m.data[9] +
                     m.data[1]*m.data[7]*m.data[10] +
                     m.data[2]*m.data[5]*m.data[11] -
                     m.data[1]*m.data[6]*m.data[11]) * invD,

                    (m.data[6]*m.data[8] - m.data[4]*m.data[10]) * invD,
                    (m.data[0]*m.data[10] - m.data[2]*m.data[8]) * invD,
                    (m.data[2]*m.data[4] - m.data[0]*m.data[6]) * invD,
                    (-m.data[3]*m.data[6]*m.data[8] +
                     m.data[3]*m.data[4]*m.data[10] +
                     m.data[2]*m.data[7]*m.data[8] -
                     m.data[0]*m.data[7]*m.data[10] -
                     m.data[2]*m.data[4]*m.data[11] +
                     m.data[0]*m.data[6]*m.data[11]) * invD,

                    (m.data[4]*m.data[9] - m.data[5]*m.data[8]) * invD,
                    (m.data[1]*m.data[8] - m.data[0]*m.data[9]) * invD,
                    (m.data[0]*m.data[5] - m.data[1]*m.data[4]) * invD,
                    (m.data[3]*m.data[5]*m.data[8] -
                     m.data[3]*m.data[4]*m.data[9] -
                     m.data[1]*m.data[7]*m.data[8] +
                     m.data[0]*m.data[7]*m.data[9] +
                     m.data[1]*m.data[4]*m.data[11] -
                     m.data[0]*m.data[5]*m.data[11]) * invD};
}

Matrix33 m33Transpose(Matrix33 m) {
  return (Matrix33){m.data[0],
                    m.data[3],
                    m.data[6],
                    m.data[1],
                    m.data[4],
                    m.data[7],
                    m.data[2],
                    m.data[5],
                    m.data[8]};
}

Matrix33 m33FromQ(Quaternion q) {
  return (Matrix33){1 - (2*q.j*q.j + 2*q.k*q.k),
                    2*q.i*q.j + 2*q.k*q.r,
                    2*q.i*q.k - 2*q.j*q.r,

                    2*q.i*q.j - 2*q.k*q.r,
                    1 - (2*q.i*q.i + 2*q.k*q.k),
                    2*q.j*q.k + 2*q.i*q.r,

                    2*q.i*q.k + 2*q.j*q.r,
                    2*q.j*q.k - 2*q.i*q.r,
                    1 - (2*q.i*q.i + 2*q.j*q.j)};
}

Matrix34 m34FromQV(Quaternion q, Vector v) {
  return (Matrix34){1 - (2*q.j*q.j + 2*q.k*q.k),
                    2*q.i*q.j + 2*q.k*q.r,
                    2*q.i*q.k - 2*q.j*q.r,
                    v.x,

                    2*q.i*q.j - 2*q.k*q.r,
                    1 - (2*q.i*q.i + 2*q.k*q.k),
                    2*q.j*q.k + 2*q.i*q.r,
                    v.y,

                    2*q.i*q.k + 2*q.j*q.r,
                    2*q.j*q.k - 2*q.i*q.r,
                    1 - (2*q.i*q.i + 2*q.j*q.j),
                    v.z};
}

/*Vector localToWorld(Vector local, Matrix34 trans) {
  return m34vMult(trans, local);
}*/

Vector worldToLocal(Vector world, Matrix34 trans) {
  Vector t = {world.x - trans.data[3], world.y - trans.data[7], world.z - trans.data[11]};
  return (Vector){t.x * trans.data[0] + t.y * trans.data[4] + t.z * trans.data[8],
                  t.x * trans.data[1] + t.y * trans.data[5] + t.z * trans.data[9],
                  t.x * trans.data[2] + t.y * trans.data[6] + t.z * trans.data[10]};
}

Vector localToWorldDir(Vector local, Matrix34 trans) {
  return (Vector){local.x * trans.data[0] + local.y * trans.data[1] + local.z * trans.data[2],
                  local.x * trans.data[4] + local.y * trans.data[5] + local.z * trans.data[6],
                  local.x * trans.data[8] + local.y * trans.data[9] + local.z * trans.data[10]};
}

Vector worldToLocalDir(Vector local, Matrix34 trans) {
  return (Vector){local.x * trans.data[0] + local.y * trans.data[4] + local.z * trans.data[8],
                  local.x * trans.data[1] + local.y * trans.data[5] + local.z * trans.data[9],
                  local.x * trans.data[2] + local.y * trans.data[6] + local.z * trans.data[10]};
}
