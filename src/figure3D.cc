#include "figure3D.hh"

#include <cmath>

#define PI 3.141592653589793238463
#define TAU 6.283185307179586476925

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
      for (int i=0; i<face.size(); i++){
        Line2D line;
        if (i != face.size()-1){
          line.p1 = doProjection(fig.points[face[i]]);
          line.p2 = doProjection(fig.points[face[i+1]]);
        }
        else {
          line.p1 = doProjection(fig.points[face[i]]);
          line.p2 = doProjection(fig.points[face[0]]);
        }
        line.c = fig.c;
        l.push_back(line);
      }      
    }
  }
  return l;
}

Figure3D createCube(const img::Color& c){
  Vector3D p0 = Vector3D::point(1, -1, -1);
  Vector3D p1 = Vector3D::point(-1, 1, -1);
  Vector3D p2 = Vector3D::point(1, 1, 1);
  Vector3D p3 = Vector3D::point(-1, -1, 1);
  Vector3D p4 = Vector3D::point(1, 1, -1);
  Vector3D p5 = Vector3D::point(-1, -1, -1);
  Vector3D p6 = Vector3D::point(1, -1, 1);
  Vector3D p7 = Vector3D::point(-1, 1, 1);

  Face f0 = { 0, 4, 2, 6 };
  Face f1 = { 4, 1, 7, 2 };
  Face f2 = { 1, 5, 3, 7 };
  Face f3 = { 5, 0, 6, 3 };
  Face f4 = { 6, 2, 7, 3 };
  Face f5 = { 0, 5, 1, 4 };

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
  Vector3D p0 = Vector3D::point(1, -1, -1);
  Vector3D p1 = Vector3D::point( -1, 1, -1 );
  Vector3D p2 = Vector3D::point( 1, 1, 1 );
  Vector3D p3 = Vector3D::point( -1, -1, 1 );
  Face f0 = { 0, 1, 2 };
  Face f1 = { 1, 3, 2 };
  Face f2 = { 0, 3, 1 };
  Face f3 = { 0, 2, 3 };

  Figure3D fig;
  fig.points.push_back(p0);
  fig.points.push_back(p1);
  fig.points.push_back(p2);
  fig.points.push_back(p3);
  fig.faces.push_back(f0);
  fig.faces.push_back(f1);
  fig.faces.push_back(f2);
  fig.faces.push_back(f3);
  fig.c = c;
  return fig;
}

Figure3D createIcosahedron(const img::Color& c) { 
  Vector3D p0 = Vector3D::point(0, 0, std::sqrt(5) / 2);
  Vector3D p1 = Vector3D::point(1, 0, 0.5);
  Vector3D p2 = Vector3D::point(std::cos(TAU / 5), std::sin(TAU / 5), 0.5);
  Vector3D p3 = Vector3D::point(std::cos(2 * TAU / 5), std::sin(2 * TAU / 5), 0.5);
  Vector3D p4 = Vector3D::point(std::cos(3 * TAU / 5), std::sin(3 * TAU / 5), 0.5);
  Vector3D p5 = Vector3D::point(std::cos(4 * TAU / 5), std::sin(4 * TAU / 5), 0.5);
  Vector3D p6 = Vector3D::point(std::cos(PI / 5), std::sin(PI / 5), -0.5);
  Vector3D p7 = Vector3D::point(std::cos(PI / 5 + TAU / 5), std::sin(PI / 5 + TAU / 5), -0.5);
  Vector3D p8 = Vector3D::point(std::cos(PI / 5 + 2 * TAU / 5), std::sin(PI / 5 + 2 * TAU / 5), -0.5);
  Vector3D p9 = Vector3D::point(std::cos(PI / 5 + 3 * TAU / 5), std::sin(PI / 5 + 3 * TAU / 5), -0.5);
  Vector3D p10 = Vector3D::point(std::cos(PI / 5 + 4 * TAU / 5), std::sin(PI / 5 + 4 * TAU / 5), -0.5);
  Vector3D p11 = Vector3D::point(0, 0, -std::sqrt(5) / 2);
              
  Face f0 = { 0, 1, 2 };
  Face f1 = { 0, 2, 3 };
  Face f2 = { 0, 3, 4 };
  Face f3 = { 0, 4, 5 };
  Face f4 = { 0, 5, 1 };
  Face f5 = { 1, 6, 2 };
  Face f6 = { 2, 6, 7 };
  Face f7 = { 2, 7, 3 };
  Face f8 = { 3, 7, 8 };
  Face f9 = { 3, 8, 4 };
  Face f10 = { 4, 8, 9 };
  Face f11 = { 4, 9, 5 };
  Face f12 = { 5, 9, 10 };
  Face f13 = { 5, 10, 1 };
  Face f14 = { 1, 10, 6 };
  Face f15 = { 11, 7, 6 };
  Face f16 = { 11, 8, 7 };
  Face f17 = { 11, 9, 8 };
  Face f18 = { 11, 10, 9 };
  Face f19 = { 11, 6, 10 };

  Figure3D fig({ p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11 },
  { f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15, f16, f17, f18, f19 }, c);

  return fig;
}

Figure3D createDodecahedron(const img::Color& c) {
  Figure3D ico = createIcosahedron(c);
  Face f0 = { 0, 1, 2, 3, 4 };
  Face f1 = { 0, 5, 6, 7, 1 };
  Face f2 = { 1, 7, 8, 9, 2 };
  Face f3 = { 2, 9, 10, 11, 3 };
  Face f4 = { 3, 11, 12, 13, 4 };
  Face f5 = { 4, 13, 14, 5, 0 };
  Face f6 = { 19, 18, 17, 16, 15 };
  Face f7 = { 19, 14, 13, 12, 18 };
  Face f8 = { 18, 12, 11, 10, 17 };
  Face f9 = { 17, 10, 9, 8, 16 };
  Face f10 = { 16, 8, 7, 6, 15 };
  Face f11 = { 15, 6, 5, 14, 19 };

  Figure3D dod({}, { f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11 }, c);
  for (Face face : ico.faces) {
    const Vector3D A = ico.points[face[0]];
    const Vector3D B = ico.points[face[1]];
    const Vector3D C = ico.points[face[2]];
    Vector3D M = (A + B + C) / 3;
    dod.points.push_back(M);
  }
  return dod;
}

Figure3D createOctahedron(const img::Color& c) {
  Vector3D p0 = Vector3D::point(1, 0, 0);
  Vector3D p1 = Vector3D::point(0, 1, 0);
  Vector3D p2 = Vector3D::point(-1, 0, 0);
  Vector3D p3 = Vector3D::point(0, -1, 0);
  Vector3D p4 = Vector3D::point(0, 0, -1);
  Vector3D p5 = Vector3D::point(0, 0, 1);  

  Face f0 = { 0, 1, 5 };
  Face f1 = { 1, 2, 5 };
  Face f2 = { 2, 3, 5 };
  Face f3 = { 3, 0, 5 };
  Face f4 = { 1, 0, 4 };
  Face f5 = { 2, 1, 4 };
  Face f6 = { 3, 2, 4 };
  Face f7 = { 0, 3, 4 };

  Figure3D fig({ p0, p1, p2, p3, p4, p5 }, { f0, f1, f2, f3, f4, f5, f6, f7 }, c);
  return fig;
}

void splitIcosahedron(Figure3D& fig) {
  std::vector<Face> newFaces;
  for (Face face : fig.faces) {
    Vector3D A = fig.points[face[0]];
    Vector3D B = fig.points[face[1]];
    Vector3D C = fig.points[face[2]];
    Vector3D D = Vector3D::point((A.x + B.x) / 2, (A.y + B.y) / 2, (A.z + B.z) / 2);
    Vector3D E = Vector3D::point((A.x + C.x) / 2, (A.y + C.y) / 2, (A.z + C.z) / 2);
    Vector3D F = Vector3D::point((B.x + C.x) / 2, (B.y + C.y) / 2, (B.z + C.z) / 2);
    fig.points.push_back(D);
    fig.points.push_back(E);
    fig.points.push_back(F);
    const int indexA = face[0];
    const int indexB = face[1];
    const int indexC = face[2];
    const int indexD = fig.points.size() - 3;
    const int indexE = fig.points.size() - 2;
    const int indexF = fig.points.size() - 1;

    Face f1 = { indexA, indexD, indexE };
    Face f2 = { indexB, indexF, indexD };
    Face f3 = { indexC, indexE, indexF };
    Face f4 = { indexD, indexF, indexE };
    newFaces.push_back(f1);
    newFaces.push_back(f2);
    newFaces.push_back(f3);
    newFaces.push_back(f4);
  }
  fig.faces = newFaces;
}

Figure3D createSphere(const int n, const img::Color& c) {
  Figure3D ico = createIcosahedron(c);
  for (int i = 0; i < n; i++) {
    splitIcosahedron(ico);
  }
  for (Vector3D& point : ico.points) {
    point.normalise();
  }
  return ico;
}

Figure3D createCone(const int n, const double h, const img::Color& c) {
  Figure3D fig;
  fig.c = c;
  Face fn, f;
  for (int i = 0; i < n; i++) {
    Vector3D p = Vector3D::point(std::cos(i * TAU / n), std::sin(i * TAU / n), 0);
    fig.points.push_back(p);
    if (i != n - 1)
      f = { i, i + 1, n };
    else
      f = { 0, i, n };
    fig.faces.push_back(f);
    fn.insert(fn.begin(), i);
  }
  fig.faces.push_back(fn);
  Vector3D top = Vector3D::point(0, 0, h);
  fig.points.push_back(top);
  return fig;
}

Figure3D createCylinder(const int n, const double h, const img::Color& c) {
  Figure3D fig;
  fig.c = c;
  for (int i = 0; i < n; i++) {
    Vector3D p = Vector3D::point(std::cos(i * TAU / n), std::sin(i * TAU / n), 0);
    fig.points.push_back(p);
  }
  for (int i = 0; i < n; i++) {
    Vector3D p = Vector3D::point(std::cos(i * TAU / n), std::sin(i * TAU / n), h);
    fig.points.push_back(p);
  }
  Face f1, f2, f;
  for (int i = 0; i < n; i++) {
    if (i != n - 1)
      f = { i, i + 1, i + n + 1, i + n };
    else
      f = { i, 0, n, i + n };
    fig.faces.push_back(f);
    f1.insert(f1.begin(), i);
    f2.push_back(i + n);
  }
  fig.faces.push_back(f1);
  fig.faces.push_back(f2);
  return fig;
}

Figure3D createTorus(const double r, const double R, const int n, const int m, const img::Color& c) {
  Figure3D fig;
  fig.c = c;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      double u = TAU * i / n;
      double v = TAU * j / m;
      double x = (R + r * std::cos(v)) * std::cos(u);
      double y = (R + r * std::cos(v)) * std::sin(u);
      double z = r * std::sin(v);
      Vector3D p = Vector3D::point(x, y, z);
      fig.points.push_back(p);
    }
  }
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      Face f = { m * i + j, m * ((i + 1) % n) + j, m * ((i + 1) % n) + (j + 1) % m, m * i + (j + 1) % m };
      fig.faces.push_back(f);
    }
  }
  return fig;
}