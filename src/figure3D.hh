#ifndef FIGURE3D_INCLUDED
#define FIGURE3D_INCLUDED
#pragma once

#include "figure2D.hh"
#include "vector.hh"
#include "easy_image.hh"
#include "zbuffer.hh"
#include "light.hh"
#include <vector>

class Point2D;
class Line2D;
class ZBuffer;
class Light;
typedef std::vector<Light> Lights3D;
typedef std::vector<Line2D> Lines2D;
typedef std::vector<int> Face;

class Figure3D {
public:
  Figure3D() {};
  Figure3D(std::vector<Vector3D> points, std::vector<Face> faces);
public:
  std::vector<Vector3D> points;
  std::vector<Face> faces;
	LightColor ambientReflection;
	LightColor diffuseReflection;
	LightColor specularReflection;
	double reflectionCoefficient;
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

Figure3D createCube();
Figure3D createTetrahedron();
Figure3D createIcosahedron();
Figure3D createDodecahedron();
Figure3D createOctahedron();
Figure3D createSphere(const int n);
Figure3D createCone(const int n, const double h);
Figure3D createCylinder(const int n, const double h);
Figure3D createTorus(const double r, const double R, const int n, const int m);
Figure3D createBuckyball();

std::vector<Face> triangulate(const Face& face);
img::EasyImage drawFigures(Figures3D& figures, int size, const img::Color& bgc, Lights3D lights);

Figures3D generateFractal(Figure3D& fig, const int nrIt, const double scale);
Vector3D normalise(Face& face);

#endif // FIGURE3D_INCLUDED
