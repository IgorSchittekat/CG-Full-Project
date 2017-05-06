#include "lSystem.hh"

#include <fstream>
#include <cmath>


#define PI 3.141592653589793238463

std::string calculateLSystem(const LParser::LSystem2D& lSystem) {
  std::string initiator = lSystem.get_initiator();
  const unsigned int nrIt = lSystem.get_nr_iterations();

  for (unsigned int i = 0; i < nrIt; i++) {
    std::string output;
    for (char character : initiator) {
      switch (character) {
      case '+': case '-': case '(': case ')':
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

std::string calculateLSystem(const LParser::LSystem3D& lSystem) {
  std::string initiator = lSystem.get_initiator();
  const unsigned int nrIt = lSystem.get_nr_iterations();

  for (unsigned int i = 0; i < nrIt; i++) {
    std::string output;
    for (char character : initiator) {
      switch (character) {
      case '+': case '-': case '(': case ')': case '^': case '&': case '\\': case '/': case '|':
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
    case '(':
      angleStack.push_back(turtleAngle);
      turtleStack.push_back(turtle);
      break;
    case ')':
      turtleAngle = angleStack.back();
      turtle = turtleStack.back();
      angleStack.pop_back();
      turtleStack.pop_back();
      break;
    default:
      if (lSystem.draw(character)) {
        Point2D p1 = turtle;
        turtle.x += std::cos(turtleAngle * PI / 180);
        turtle.y += std::sin(turtleAngle * PI / 180);
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

Figure3D drawLSystem(std::string lString, const LParser::LSystem3D& lSystem){
  Figure3D fig;
  const std::set<char> alphabet = lSystem.get_alphabet();
  const double angle = lSystem.get_angle();
  Vector3D turtle = Vector3D::point(0, 0, 0);
  Vector3D H = Vector3D::vector(1, 0, 0);
  Vector3D L = Vector3D::vector(0, 1, 0);
  Vector3D U = Vector3D::vector(0, 0, 1);
  Vector3D HNew, LNew, UNew;
  std::vector<Vector3D> angleStack;
  std::vector<Vector3D> turtleStack;
  for (char character:lString) {
    switch (character) {
    	case '+':
        HNew = H * std::cos(angle * PI / 180) + L * std::sin(angle * PI / 180);
        LNew = -H * std::sin(angle * PI / 180) + L * std::cos(angle * PI / 180);
        H = HNew;
        L = LNew;
        break;
    	case '-':
    		HNew = H * std::cos(-angle * PI / 180) + L * std::sin(-angle * PI / 180);
        LNew = -H * std::sin(-angle * PI / 180) + L * std::cos(-angle * PI / 180);
        H = HNew;
        L = LNew;
        break;
    	case '^':
        HNew = H * std::cos(angle * PI / 180) + U * std::sin(angle * PI / 180);
        UNew = -H * std::sin(angle * PI / 180) + U * std::cos(angle * PI / 180);
    		H = HNew;
        U = UNew;
        break;
    	case '&':
        HNew = H * std::cos(-angle * PI / 180) + U * std::sin(-angle * PI / 180);
        UNew = -H * std::sin(-angle * PI / 180) + U * std::cos(-angle * PI / 180);
    		H = HNew;
        U = UNew;
        break;
    	case '\\':
        LNew = L * std::cos(angle * PI / 180) - U * std::sin(angle * PI / 180);
        UNew = L * std::sin(angle * PI / 180) + U * std::cos(angle * PI / 180);
    		L = LNew;
        U = UNew;
        break;
    	case '/':
        LNew = L * std::cos(-angle * PI / 180) - U * std::sin(-angle * PI / 180);
        UNew = L * std::sin(-angle * PI / 180) + U * std::cos(-angle * PI / 180);
    		L = LNew;
        U = UNew;
        break;
    	case '|':
    		H = -H;
        L = -L;
        break;
      case '(':
        angleStack.push_back(H);
        angleStack.push_back(L);
        angleStack.push_back(U);
        turtleStack.push_back(turtle);
        break;
      case ')':
        U = angleStack.back();
        angleStack.pop_back();
        L = angleStack.back();
        angleStack.pop_back();
        H = angleStack.back();
        angleStack.pop_back();
        turtle = turtleStack.back();
        turtleStack.pop_back();
        break;
      default: {
      	if (lSystem.draw(character)) {
          Vector3D p1 = turtle;
          turtle += H;
          Face f = {(int) fig.points.size(), (int) fig.points.size() + 1};
          fig.faces.push_back(f);
          fig.points.push_back(p1);
          fig.points.push_back(turtle);
        }
        else {
          turtle += H;
        }
      }
    }
  }
  return fig;
}

img::EasyImage LSystems2D(int size, const std::vector<double> &bgColor, const std::string &inFile, const std::vector<double> &color) {
  LParser::LSystem2D lSystem;
  std::ifstream file(inFile);
  if (!file.is_open()) {
    std::cerr << "ERROR: File " << inFile << " not found!" << std::endl;
    return img::EasyImage();
  }
  file >> lSystem;
  file.close();

  std::string lString = calculateLSystem(lSystem);
  img::Color c(color[0] * 255, color[1] * 255, color[2] * 255);
  img::Color bgc(bgColor[0] * 255, bgColor[1] * 255, bgColor[2] * 255);
  return drawLSystem(lString, lSystem, c, size, bgc);
}

Figure3D LSystems3D(const std::string &inFile){
  LParser::LSystem3D lSystem;
  std::ifstream file(inFile);
  if (!file.is_open()) {
    std::cerr << "ERROR: File " << inFile << " not found!" << std::endl;
    return Figure3D();
  }
  file >> lSystem;
  file.close();

  std::string lString = calculateLSystem(lSystem);
  return drawLSystem(lString, lSystem);
}