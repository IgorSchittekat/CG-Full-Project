#ifndef FIGURE3D_INCLUDED
#define FIGURE3D_INCLUDED
#pragma once

#include "figure2D.hh"
#include "vector.hh"
#include "easy_image.hh"
#include "zbuffer.hh"
#include <vector>
#pragma once

class Point2D;
class Line2D;
class ZBuffer;
typedef std::vector<Line2D> Lines2D;
typedef std::vector<int> Face;

class Figure3D {
public:
  Figure3D() {};
  Figure3D(std::vector<Vector3D> points, std::vector<Face> faces, img::Color c);
public:
  std::vector<Vector3D> points;
  std::vector<Face> faces;
  img::Color c;
};

typedef std::vector<Figure3D> Figures3D;

Matrix scale(const double schale);
Matrix rotateX(const double angle);
Matrix rotateY(const double angle);
Matrix rotateZ(const double angle);
Matrix shift(const Vector3D& vector);

void toPolar(const Vector3D& point, double& theta, double& phi, double& r);
Matrix eyePointTrans(const Vector3D& eyepoint);
void applyTransformation(Figure3D& fig, const Matrix& m);
void applyTransformation(Figures3D& figures, const Matrix& m);

Point2D doProjection(const Vector3D& point, const double d = 1);
Lines2D doProjection(const Figures3D& figures);

Figure3D createCube(const img::Color& c);
Figure3D createTetrahedron(const img::Color& c);
Figure3D createIcosahedron(const img::Color& c);
Figure3D createDodecahedron(const img::Color& c);
Figure3D createOctahedron(const img::Color& c);
Figure3D createSphere(const int n, const img::Color& c);
Figure3D createCone(const int n, const double h, const img::Color& c);
Figure3D createCylinder(const int n, const double h, const img::Color& c);
Figure3D createTorus(const double r, const double R, const int n, const int m, const img::Color& c);

std::vector<Face> triangulate(const Face& face);
img::EasyImage drawFigures(Figures3D& figures, int size, const img::Color& bgc);

#endif // FIGURE3D_INCLUDED
