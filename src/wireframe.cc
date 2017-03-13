#include "wireframe.hh"

void transform(Figure3D& fig, double scaleFactor, double angleX, double angleY,
  double angleZ, const Vector3D& center) {
  Matrix M = scale(scaleFactor);
  
  if (angleX != 0)
    M *= rotateX(angleX);
  if (angleY != 0)
    M *= rotateY(angleY);
  if (angleZ != 0)
    M *= rotateZ(angleZ);
  M *= shift(center);
  applyTransformation(fig, M);
}

Figure3D calculateFigure(const std::string& figureName, const ini::Configuration &configuration) {
  Figure3D figure;
  const std::string type = configuration[figureName]["type"].as_string_or_die();
  if (type == "LineDrawing") {
    const double scale = configuration[figureName]["scale"].as_double_or_die();
    const double rotateX = configuration[figureName]["rotateX"].as_double_or_die();
    const double rotateY = configuration[figureName]["rotateY"].as_double_or_die();
    const double rotateZ = configuration[figureName]["rotateZ"].as_double_or_die();
    const std::vector<double> center = configuration[figureName]["center"].as_double_tuple_or_die();
    const std::vector<double> color = configuration[figureName]["color"].as_double_tuple_or_die();
    const unsigned int nrPoints = configuration[figureName]["nrPoints"].as_int_or_die();
    const unsigned int nrLines = configuration[figureName]["nrLines"].as_int_or_die();
    for (unsigned int i = 0; i < nrPoints; i++) {
      std::vector<double> point = configuration[figureName]["point" + std::to_string(i)].as_double_tuple_or_die();
      Vector3D p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      figure.points.push_back(p);
    }
    for (unsigned int i = 0; i < nrLines; i++) {
      std::vector<int> line = configuration[figureName]["line" + std::to_string(i)].as_int_tuple_or_die();
      Face f;
      f.point_indexes = { line[0], line[1] };
      figure.faces.push_back(f);
    }
    Vector3D centerVec;
    centerVec.point(center[0], center[1], center[2]);
    transform(figure, scale, rotateX, rotateY, rotateZ, centerVec);
    img::Color c(color[0] * 255, color[1] * 255, color[2] * 255);
    figure.c = c;
  }
  else if (type == "Cube") {
    const std::vector<double> color = configuration[figureName]["color"].as_double_tuple_or_die();
    const double scale = configuration[figureName]["scale"].as_double_or_die();
    const double rotateX = configuration[figureName]["rotateX"].as_double_or_die();
    const double rotateY = configuration[figureName]["rotateY"].as_double_or_die();
    const double rotateZ = configuration[figureName]["rotateZ"].as_double_or_die();
    const std::vector<double> center = configuration[figureName]["center"].as_double_tuple_or_die();
    img::Color c(color[0] * 255, color[1] * 255, color[2] * 255);
    figure = createCube(c);
    Vector3D centerVec;
    centerVec.point(center[0], center[1], center[2]);
    transform(figure, scale, rotateX, rotateY, rotateZ, centerVec);
  }
  return figure;
}