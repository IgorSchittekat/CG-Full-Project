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
