#include "wireframe.hh"
#include "l_parser.hh"
#include "lSystem.hh"
#include <iostream>

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

Figures3D calculateFigure(const std::string& figureName, const ini::Configuration &configuration) {
  Figure3D figure;
  const std::string type = configuration[figureName]["type"].as_string_or_die();
  const double scale = configuration[figureName]["scale"].as_double_or_die();
  const double rotateX = configuration[figureName]["rotateX"].as_double_or_die();
  const double rotateY = configuration[figureName]["rotateY"].as_double_or_die();
  const double rotateZ = configuration[figureName]["rotateZ"].as_double_or_die();
	const std::vector<double> center = configuration[figureName]["center"].as_double_tuple_or_die();
	Vector3D centerVec = Vector3D::point(center[0], center[1], center[2]);
	LightColor ambientReflection;
	configuration[figureName]["color"].as_double_tuple_if_exists(ambientReflection);
	ambientReflection = configuration[figureName]["ambientReflection"].as_double_tuple_or_default(ambientReflection);
	configuration[figureName]["diffuseReflection"].as_double_tuple_if_exists(figure.diffuseReflection);
	configuration[figureName]["specularReflection"].as_double_tuple_if_exists(figure.specularReflection);
  if (type == "LineDrawing") {   
    const uint nrPoints = configuration[figureName]["nrPoints"].as_int_or_die();
    const uint nrLines = configuration[figureName]["nrLines"].as_int_or_die();
    for (uint i = 0; i < nrPoints; i++) {
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
      f = { line[0], line[1] };
      figure.faces.push_back(f);
    }
  }
  else if (type == "Cube") {
    figure = createCube();
  }
  else if (type == "Tetrahedron") {
    figure = createTetrahedron();
  }
  else if (type == "Icosahedron") {
    figure = createIcosahedron();
  }
  else if (type == "Dodecahedron") {
    figure = createDodecahedron();
  }
  else if (type == "Cone") {
    const unsigned int n = configuration[figureName]["n"].as_int_or_die();
    const double h = configuration[figureName]["height"].as_double_or_die();
    figure = createCone(n, h);
  }
    else if (type == "Cylinder") {
    const unsigned int n = configuration[figureName]["n"].as_int_or_die();
    const double h = configuration[figureName]["height"].as_double_or_die();
    figure = createCylinder(n, h);
  }
  else if (type == "Sphere") {
    const unsigned int n = configuration[figureName]["n"].as_int_or_die();
    figure = createSphere(n);
  }
  else if (type == "Torus"){
    const double r = configuration[figureName]["r"].as_double_or_die();
    const double R = configuration[figureName]["R"].as_double_or_die();
    const unsigned int m = configuration[figureName]["m"].as_int_or_die();
    const unsigned int n = configuration[figureName]["n"].as_int_or_die();
    figure = createTorus(r, R, m, n);
  }
  else if (type == "Octahedron"){
    figure = createOctahedron();
  }
  else if (type == "3DLSystem") {
    const std::string inFile = configuration[figureName]["inputfile"].as_string_or_die();
    figure = LSystems3D(inFile);
  }
	else if (type == "BuckyBall") {
		figure = createBuckyball();
	}
	else if (type.substr(0, 7) == "Fractal") {
		const int nrIt = configuration[figureName]["nrIterations"].as_int_or_die();
		const double fractalScale = configuration[figureName]["fractalScale"].as_double_or_die();
		if (type == "FractalTetrahedron") {
			figure = createTetrahedron();
		}
		else if (type == "FractalCube") {
			figure = createCube();
		}
		else if (type == "FractalIcosahedron") {
			figure = createIcosahedron();
		}
		else if (type == "FractalOctahedron") {
			figure = createOctahedron();
		}
		else if (type == "FractalDodecahedron") {
			figure = createDodecahedron();
		}
		else if (type == "FractalBuckyBall") {
			figure = createBuckyball();
		}
		else {
			return Figures3D();
		}
		figure.ambientReflection = ambientReflection;
		Figures3D figures = generateFractal(figure, nrIt, fractalScale);
		for (Figure3D& fig : figures) {
			transform(fig, scale, rotateX, rotateY, rotateZ, centerVec);
		}
		return figures;
	}
	else {
    return Figures3D();
  }
	figure.ambientReflection = ambientReflection;
  transform(figure, scale, rotateX, rotateY, rotateZ, centerVec);
	Figures3D figures;
	figures.push_back(figure);
  return figures;
}