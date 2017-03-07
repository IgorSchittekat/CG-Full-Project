#include "figure3D.hh"

#include <cmath>

#define PI 3.141592653589793238463

Matrix scale(const double scaleFactor) {
  Matrix m;
  m(1, 1) = scaleFactor;
  m(2, 2) = scaleFactor;
  m(3, 3) = scaleFactor;
  return m;
}

Matrix rotateX(const double angle) {
  Matrix m;
  m(2, 2) = std::cos(angle * PI / 180);
  m(2, 3) = std::sin(angle * PI / 180);
  m(2, 3) = -std::sin(angle * PI / 180);
  m(3, 3) = std::cos(angle * PI / 180);

  return m;
}

Matrix rotateY(const double angle) {
  Matrix m;
  m(1, 1) = std::cos(angle * PI / 180);
  m(1, 3) = -std::sin(angle * PI / 180);
  m(3, 1) = std::sin(angle * PI / 180);
  m(3, 3) = std::cos(angle * PI / 180);

  return m;
}

Matrix rotateZ(const double angle) {
  Matrix m;
  m(1, 1) = std::cos(angle * PI / 180);
  m(1, 2) = std::sin(angle * PI / 180);
  m(2, 1) = -std::sin(angle * PI / 180);
  m(2, 2) = std::cos(angle * PI / 180);

  return m;
}

Matrix shift(const Vector3D & vec) {
  Matrix m;
  m(4, 1) = vec.x;
  m(4, 2) = vec.y;
  m(4, 3) = vec.z;
  return m;
}

void toPolar(const Vector3D& p, double &theta, double &phi, double &r) {
  r = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
  theta = std::atan2(p.y, p.x);
  phi = std::acos(p.z/r);
}

Matrix eyePointTrans(const Vector3D& eyepoint) {
  Matrix m;
  double theta, phi, r;
  toPolar(eyepoint, theta, phi, r);
  m(1, 1) = -std::sin(theta);
  m(1, 2) = -std::cos(theta) * std::cos(phi);
  m(1, 3) = std::cos(theta) * std::sin(phi);
  m(2, 1) = std::cos(theta);
  m(2, 2) = -std::sin(theta) * std::cos(phi);
  m(2, 3) = std::sin(theta) * std::sin(phi);
  m(3, 2) = std::sin(phi);
  m(3, 3) = std::cos(phi);
  m(4, 3) = -r;
  return m;
}

void applyTransformation(Figure3D & fig, const Matrix & m) {
  for (Vector3D& point : fig.points) {
    point *= m;
  }
}

void applyTransformation(Figures3D & figures, const Matrix & m) {
  for (Figure3D& fig : figures) {
    applyTransformation(fig, m);
  }
}

Point2D doProjection(const Vector3D & point, const double d=1) {
  Point2D p;
  p.x = (d * point.x) / -point.z;
  p.y = (d * point.y) / -point.z;
  return p;
}

Lines2D doProjection(const Figures3D & figures) {
  Lines2D l;
  for (const Figure3D& fig : figures) {
    for (const Face& face : fig.faces) {
      Line2D line;
      line.p1 = doProjection(fig.points[face.point_indexes[0]]);
      line.p2 = doProjection(fig.points[face.point_indexes[1]]);
      line.c = fig.c;
      l.push_back(line);
    }
  }
  return l;
}

