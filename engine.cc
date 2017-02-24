#include "easy_image.hh"
#include "ini_configuration.hh"
#include "helpers.hh"
#include "l_parser.hh"

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


img::EasyImage generate_image(const ini::Configuration &configuration)
{
	std::string type = configuration["General"]["type"].as_string_or_die();
	if (type == "2DLSystem")
	{
		const unsigned int size = configuration["General"]["size"].as_int_or_die();
		const std::vector<double> bgColor = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
		const std::string inFile = configuration["2DLSystem"]["inputfile"].as_string_or_die();
		const std::vector<double> color = configuration["2DLSystem"]["color"].as_double_tuple_or_die();
		return LSystems2D(size, bgColor, inFile, color);
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
