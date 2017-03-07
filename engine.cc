#include "easy_image.hh"
#include "ini_configuration.hh"
#include "helpers.hh"
#include "l_parser.hh"
#include "figure3D.hh"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cmath>
#include <assert.h>

#define PI 3.141592653589793238463

std::string calculateLSystem(const LParser::LSystem2D& lSystem) {
	std::string initiator = lSystem.get_initiator();
	const unsigned int nrIt = lSystem.get_nr_iterations();
	
	for (unsigned int i = 0; i < nrIt; i++) {
		std::string output;
		for (char character : initiator) {
      switch (character) {
      case '+': case '-': case '(': case ')': case '[': case ']':
        output += character;
        break;
      default:
        output += lSystem.get_replacement(character);
      }
		}
		initiator = output;
	}
	return initiator;
}

img::EasyImage drawLSystem(std::string lString, const LParser::LSystem2D& lSystem, const img::Color& c, int size, const img::Color& bgc) {
	const std::set<char> alphabet = lSystem.get_alphabet();
	const double angle = lSystem.get_angle();
	const double startingAngle = lSystem.get_starting_angle();
	Point2D turtle = { 0, 0 };
	Lines2D lines;
  std::vector<double> angleStack;
  std::vector<Point2D> turtleStack;
	double turtleAngle = startingAngle;
	for (char character : lString) {
    switch (character) {
    case '+':
      turtleAngle += angle;
      break;
    case '-':
      turtleAngle -= angle;
      break;
    case '(': case '[':
      angleStack.push_back(turtleAngle);
      turtleStack.push_back(turtle);
      break;
    case ')': case ']':
      turtleAngle = angleStack.back();
      turtle = turtleStack.back();
      angleStack.pop_back();
      turtleStack.pop_back();
      break;
    default:
      if (lSystem.draw(character)) {
        Point2D p1 = turtle;
        turtle.x += std::cos(turtleAngle * PI / 180.0);
        turtle.y += std::sin(turtleAngle * PI / 180.0);
        lines.push_back({ p1, turtle, c });
      }
      else {
        turtle.x += std::cos(turtleAngle * PI / 180);
        turtle.y += std::sin(turtleAngle * PI / 180);
      }
    }
	}
  return draw2DLines(lines, size, bgc);
}

img::EasyImage LSystems2D(int size, const std::vector<double> &bgColor, const std::string &inFile, const std::vector<double> &color) {
	LParser::LSystem2D lSystem;
	std::ifstream file(inFile);
	file >> lSystem;
	file.close();

	std::string lString = calculateLSystem(lSystem);
	img::Color c(color[0] * 255, color[1] * 255, color[2] * 255);
	img::Color bgc(bgColor[0] * 255, bgColor[1] * 255, bgColor[2] * 255);
	return drawLSystem(lString, lSystem, c, size, bgc);
}

void transform(Figure3D& fig, double scaleFactor, double angleX, double angleY, double angleZ, Vector3D& eye, const Vector3D& center) {
  Matrix M = scale(scaleFactor);
  
  if (angleX != 0)
    M *= rotateX(angleX);
  if (angleY != 0)
    M *= rotateY(angleY);
  if (angleZ != 0)
    M *= rotateZ(angleZ);
  M *= shift(center);
  M *= eyePointTrans(eye);
  applyTransformation(fig, M);
}

Figure3D calculateFigure(const std::string& figureName, const ini::Configuration &configuration, Vector3D& eye) {
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
    centerVec.x = center[0];
    centerVec.y = center[1];
    centerVec.z = center[2];
    transform(figure, scale, rotateX, rotateY, rotateZ, eye, centerVec);
    img::Color c(color[0] * 255, color[1] * 255, color[2] * 255);
    figure.c = c;
  }
  return figure;
}

img::EasyImage generate_image(const ini::Configuration &configuration)
{
	const std::string type = configuration["General"]["type"].as_string_or_die();
	if (type == "2DLSystem") {
		const unsigned int size = configuration["General"]["size"].as_int_or_die();
		const std::vector<double> bgColor = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
		const std::string inFile = configuration["2DLSystem"]["inputfile"].as_string_or_die();
		const std::vector<double> color = configuration["2DLSystem"]["color"].as_double_tuple_or_die();
		return LSystems2D(size, bgColor, inFile, color);
	}
  else if (type == "Wireframe") {
    const unsigned int size = configuration["General"]["size"].as_int_or_die();
    const std::vector<double> bgColor = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
    const unsigned int nrFigures = configuration["General"]["nrFigures"].as_int_or_die();
    const std::vector<double> eye = configuration["General"]["eye"].as_double_tuple_or_die();
    Figures3D figures;
    for (unsigned int i = 0; i < nrFigures; i++) {
      const std::string figureName = "Figure" + std::to_string(i);
      Vector3D eyePoint;
      eyePoint.x = eye[0];
      eyePoint.y = eye[1];
      eyePoint.z = eye[2];
      figures.push_back(calculateFigure(figureName, configuration, eyePoint));
    }
    img::Color c(bgColor[0] * 255, bgColor[1] * 255, bgColor[2] * 255);
    Lines2D lines = doProjection(figures);
    return draw2DLines(lines, size, c);
  }
	
	return img::EasyImage();
}

int main(int argc, char const* argv[])
{
        int retVal = 0;
        try
        {
                for(int i = 1; i < argc; ++i)
                {
                        ini::Configuration conf;
                        try
                        {
                                std::ifstream fin(argv[i]);
                                fin >> conf;
                                fin.close();
                        }
                        catch(ini::ParseException& ex)
                        {
                                std::cerr << "Error parsing file: " << argv[i] << ": " << ex.what() << std::endl;
                                retVal = 1;
                                continue;
                        }

                        img::EasyImage image = generate_image(conf);
                        if(image.get_height() > 0 && image.get_width() > 0)
                        {
                                std::string fileName(argv[i]);
                                std::string::size_type pos = fileName.rfind('.');
                                if(pos == std::string::npos)
                                {
                                        //filename does not contain a '.' --> append a '.bmp' suffix
                                        fileName += ".bmp";
                                }
                                else
                                {
                                        fileName = fileName.substr(0,pos) + ".bmp";
                                }
                                try
                                {
                                        std::ofstream f_out(fileName.c_str(),std::ios::trunc | std::ios::out | std::ios::binary);
                                        f_out << image;

                                }
                                catch(std::exception& ex)
                                {
                                        std::cerr << "Failed to write image to file: " << ex.what() << std::endl;
                                        retVal = 1;
                                }
                        }
                        else
                        {
                                std::cout << "Could not generate image for " << argv[i] << std::endl;
                        }
                }
        }
        catch(const std::bad_alloc &exception)
        {
    		//When you run out of memory this exception is thrown. When this happens the return value of the program MUST be '100'.
    		//Basically this return value tells our automated test scripts to run your engine on a pc with more memory.
    		//(Unless of course you are already consuming the maximum allowed amount of memory)
    		//If your engine does NOT adhere to this requirement you risk losing points because then our scripts will
		//mark the test as failed while in reality it just needed a bit more memory
                std::cerr << "Error: insufficient memory" << std::endl;
                retVal = 100;
        }
        return retVal;
}
