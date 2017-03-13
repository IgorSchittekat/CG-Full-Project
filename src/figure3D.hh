#ifndef FIGURE3D_INCLUDED
#define FIGURE3D_INCLUDED

#include "figure2D.hh"
#include "vector.hh"
#include "easy_image.hh"

#include <vector>

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

Point2D doProjection(const Vector3D& point, const double d);
Lines2D doProjection(const Figures3D& figures);

Figure3D createCube(const img::Color& c);
Figure3D createTetrahedron(const img::Color& c);
Figure3D createIcosahedron(const img::Color& c);
Figure3D createDodecahedron(const img::Color& c);
Figure3D createSphere(const int n, const img::Color& c, const double radius);

#endif // FIGURE3D_INCLUDED