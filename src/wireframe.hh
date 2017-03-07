#ifndef WIREFRAME_INCLUDED
#define WIREFRAME_INCLUDED

#include "ini_configuration.hh"
#include "figure3D.hh"
#include "figure2D.hh"


void transform(Figure3D& fig, double scaleFactor, double angleX, double angleY,
  double angleZ, Vector3D& eye, const Vector3D& center);
Figure3D calculateFigure(const std::string& figureName, const ini::Configuration &configuration, Vector3D& eye);




#endif //WIREFRAME_INCLUDED
