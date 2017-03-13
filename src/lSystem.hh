#ifndef L_SYSTEM_INCLUDED
#define L_SYSTEM_INCLUDED

#include "l_parser.hh"
#include "easy_image.hh"
#include "figure2D.hh"

std::string calculateLSystem(const LParser::LSystem2D& lSystem);
img::EasyImage drawLSystem(std::string lString, const LParser::LSystem2D& lSystem, 
  const img::Color& c, int size, const img::Color& bgc);
img::EasyImage LSystems2D(int size, const std::vector<double> &bgColor, 
  const std::string &inFile, const std::vector<double> &color);

#endif //L_SYSTEM_INCLUDED