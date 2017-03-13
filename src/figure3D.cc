#include "figure3D.hh"

#include <cmath>

#define PI 3.141592653589793238463
#define TAU 6.283185307179586476925

Face::Face(std::vector<int> point_indexes) :
  point_indexes(point_indexes) {
}

Figure3D::Figure3D(std::vector<Vector3D> points, std::vector<Face> faces, img::Color c) :
  points(points),
  faces(faces),
  c(c) {
}

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
  m(3, 2) = -std::sin(angle * PI / 180);
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
  r = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2));
  theta = std::atan2(p.y, p.x); // in rad
  phi = std::acos(p.z / r); // in rad
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

Figure3D createCube(const img::Color& c){
  Vector3D p0, p1, p2, p3, p4, p5, p6, p7;
  p0.x = 1; p0.y = -1, p0.z = -1;
  p1.x = -1; p1.y = 1, p1.z = -1;
  p2.x = 1; p2.y = 1, p2.z = 1;
  p3.x = -1; p3.y = -1, p3.z = 1;
  p4.x = 1; p4.y = 1, p4.z = -1;
  p5.x = -1; p5.y = -1, p5.z = -1;
  p6.x = 1; p6.y = -1, p6.z = 1;
  p7.x = -1; p7.y = 1, p7.z = 1;

  Face f0({ 0, 4, 2, 6 });
  Face f1({ 4, 1, 7, 2 });
  Face f2({ 1, 5, 3, 7 });
  Face f3({ 5, 0, 6, 3 });
  Face f4({ 6, 2, 7, 3 });
  Face f5({ 0, 5, 1, 4 });

  Figure3D fig;
  fig.points.push_back(p0);
  fig.points.push_back(p1);
  fig.points.push_back(p2);
  fig.points.push_back(p3);
  fig.points.push_back(p4);
  fig.points.push_back(p5);
  fig.points.push_back(p6);
  fig.points.push_back(p7);
  fig.faces.push_back(f0);
  fig.faces.push_back(f1);
  fig.faces.push_back(f2);
  fig.faces.push_back(f3);
  fig.faces.push_back(f4);
  fig.faces.push_back(f5);
  fig.c = c;
  return fig;
}

Figure3D createTetrahedron(const img::Color& c) {
  Vector3D p1, p2, p3, p4;
  p1.x = 1; p1.y = -1; p1.z = -1;
  p2.x = -1; p2.y = 1; p2.z = -1;
  p3.x = 1; p3.y = 1; p3.z = 1;
  p4.x = -1; p4.y = -1; p4.z = 1;
  //p1.point(1, -1, -1);
  //p2.point( -1, 1, -1 );
  //p3.point( 1, 1, 1 );
  //p4.point( -1, -1, 1 );
  Face f1({ 1, 2, 3 });
  Face f2({ 2, 4, 3 });
  Face f3({ 1, 4, 2 });
  Face f4({ 1, 3, 4 });

  Figure3D fig;
  fig.points.push_back(p1);
  fig.points.push_back(p2);
  fig.points.push_back(p3);
  fig.points.push_back(p4);
  fig.faces.push_back(f1);
  fig.faces.push_back(f2);
  fig.faces.push_back(f3);
  fig.faces.push_back(f4);
  fig.c = c;
  for (Vector3D vec : fig.points) {
    vec.print(std::cout);
  }
  std::cout << std::endl;
  return fig;
}

Figure3D createIcosahedron(const img::Color& c) { 
  Vector3D p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12;
  p1.point(0, 0, std::sqrt(5) / 2);
  p2.point(1, 0, 0.5);
  p3.point(std::cos(TAU / 5), std::sin(TAU / 5), 0.5);
  p4.point(std::cos(2 * TAU / 5), std::sin(2 * TAU / 5), 0.5);
  p5.point(std::cos(3 * TAU / 5), std::sin(3 * TAU / 5), 0.5);
  p6.point(std::cos(4 * TAU / 5), std::sin(4 * TAU / 5), 0.5);
  p7.point(std::cos(PI / 5), std::sin(PI / 5), -0.5);
  p8.point(std::cos(PI / 5 + TAU / 5), std::sin(PI / 5 + TAU / 5), -0.5);
  p9.point(std::cos(PI / 5 + 2 * TAU / 5), std::sin(PI / 5 + 2 * TAU / 5), -0.5);
  p10.point(std::cos(PI / 5 + 3 * TAU / 5), std::sin(PI / 5 + 3 * TAU / 5), -0.5);
  p11.point(std::cos(PI / 5 + 4 * TAU / 5), std::sin(PI / 5 + 4 * TAU / 5), -0.5);
  p12.point(0, 0, -std::sqrt(5) / 2);
              
  Face f1({ 1, 2, 3 });
  Face f2({ 1, 3, 4 });
  Face f3({ 1, 4, 5 });
  Face f4({ 1, 5, 6 });
  Face f5({ 1, 6, 2 });
  Face f6({ 2, 7, 3 });
  Face f7({ 3, 7, 8 });
  Face f8({ 3, 8, 4 });
  Face f9({ 4, 8, 9 });
  Face f10({ 4, 9, 5 });
  Face f11({ 5, 9, 10 });
  Face f12({ 5, 10, 6 });
  Face f13({ 6, 10, 11 });
  Face f14({ 6, 11, 2 });
  Face f15({ 2, 11, 7 });
  Face f16({ 12, 8, 7 });
  Face f17({ 12, 9, 8 });
  Face f18({ 12, 10, 9 });
  Face f19({ 12, 11, 10 });
  Face f20({ 12, 7, 11 });

  Figure3D fig({ p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12 },
  { f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15, f16, f17, f18, f19, f20 }, c);

  return fig;
}

Figure3D createDodecahedron(const img::Color& c) {
  Figure3D ico = createIcosahedron(c);
  Face f1({ 1, 2, 3, 4, 5 });
  Face f2({ 1, 6, 7, 8, 2 });
  Face f3({ 2, 8, 9, 10, 3 });
  Face f4({ 3, 10, 11, 12, 4 });
  Face f5({ 4, 12, 13, 14, 5 });
  Face f6({ 5, 14, 15, 6, 1 });
  Face f7({ 20, 19, 18, 17, 16 });
  Face f8({ 20, 15, 14, 13, 19 });
  Face f9({ 19, 13, 12, 11, 18 });
  Face f10({ 18, 11, 10, 9, 17 });
  Face f11({ 17, 9, 8, 7, 16 });
  Face f12({ 16, 7, 6, 15, 20 });

  Figure3D dod({}, { f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12 }, c);
  for (Face face : ico.faces) {
    const Vector3D A = ico.points[face.point_indexes[0]];
    const Vector3D B = ico.points[face.point_indexes[1]];
    const Vector3D C = ico.points[face.point_indexes[2]];
    Vector3D M = (A + B + C) / 3;
    dod.points.push_back(M);
  }
  return dod;
}

void splitIcosahedron(Figure3D& fig) {
  std::vector<Face> newFaces;
  for (Face face : fig.faces) {
    Vector3D A = fig.points[face.point_indexes[0]];
    Vector3D B = fig.points[face.point_indexes[1]];
    Vector3D C = fig.points[face.point_indexes[2]];
    Vector3D D, E, F;
    D.point((A.x + B.x) / 2, (A.y + B.y) / 2, (A.z + B.z) / 2);
    E.point((A.x + C.x) / 2, (A.y + C.y) / 2, (A.z + C.z) / 2);
    F.point((B.x + C.x) / 2, (B.y + C.y) / 2, (B.z + C.z) / 2);
    fig.points.push_back(D);
    fig.points.push_back(E);
    fig.points.push_back(F);
    const int indexA = face.point_indexes[0];
    const int indexB = face.point_indexes[1];
    const int indexC = face.point_indexes[2];
    const int indexD = fig.points.size() - 3;
    const int indexE = fig.points.size() - 2;
    const int indexF = fig.points.size() - 1;

    Face f1, f2, f3, f4;
    f1.point_indexes = { indexA, indexD, indexE };
    f2.point_indexes = { indexB, indexF, indexD };
    f3.point_indexes = { indexC, indexE, indexF };
    f4.point_indexes = { indexD, indexF, indexE };
    newFaces.push_back(f1);
    newFaces.push_back(f2);
    newFaces.push_back(f3);
    newFaces.push_back(f4);
  }
  fig.faces = newFaces;
}

Figure3D createSphere(const int n, const img::Color& c, const double radius) {
  Figure3D ico = createIcosahedron(c);
  for (int i = 0; i < n; i++) {
    splitIcosahedron(ico);
  }
  for (Vector3D point : ico.points) {
    point.normalise();
  }
  return ico;
}


