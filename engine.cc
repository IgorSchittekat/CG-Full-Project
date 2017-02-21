#include "easy_image.hh"
#include "ini_configuration.hh"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cmath>
#include <assert.h>

inline int roundToInt(const double d) {
	return (d < 0) ? (int)std::ceil(d - 0.5) : (int)std::floor(d + 0.5);
}

img::EasyImage ColorRectangle(const unsigned int width, const unsigned int height)
{
	img::EasyImage image(width, height);
	for (unsigned int x = 0; x < width; x++)
	{
		for (unsigned int y = 0; y < height; y++)
		{
			img::Color c(roundToInt(255.0 * (double)x / (double)width),
				roundToInt(255.0 * (double)y / (double)height), 
				roundToInt((255.0 * (double)x / width) + (255.0 * (double)y / height)));

			image(x, y) = c;
		}
	}
	return image;
}

img::EasyImage Blocks(const unsigned int width, const unsigned int height, const std::vector<double> &cWhite, const std::vector<double> &cBlack,
	const unsigned int nrXBlocks, const unsigned int ntYBlocks, const bool invC)
{
	img::EasyImage image(width, height);
	const unsigned int widthBlock = width / nrXBlocks;
	const unsigned int heightBlock = height / ntYBlocks;
	for (unsigned int x = 0; x < width; x++)
	{
		for (unsigned int y = 0; y < height; y++)
		{
			img::Color c1(cWhite[0] * 255, cWhite[1] * 255, cWhite[2] * 255);
			img::Color c2(cBlack[0] * 255, cBlack[1] * 255, cBlack[2] * 255);

			int blockX = x / widthBlock;
			int blockY = y / heightBlock;
			if ((blockX + blockY) % 2 == 0)
			{
				if (!invC) image(x, y) = c1;
				else image(x, y) = c2;
			}
			else
			{
				if (!invC) image(x, y) = c2;
				else image(x, y) = c1;
			}
		}
	}
	return image;
}

img::EasyImage Lines(const unsigned int width, const unsigned int height, const std::string &figure, const std::vector<double> bgColor, 
	const std::vector<double> &lineColor, const int nrLines)
{
	img::Color cBg(bgColor[0] * 255, bgColor[1] * 255, bgColor[2] * 255);
	img::Color cLine(lineColor[0] * 255, lineColor[1] * 255, lineColor[2] * 255);
	img::EasyImage image(width, height, cBg);
	assert(width == height);
	const unsigned int sidePoints = roundToInt((double)width / (double)nrLines);
	if (figure == "QuarterCircle" || figure == "Eye")
	{
		for (unsigned int i = 0; i < width; i += sidePoints)
		{
			image.draw_line(i, height - 1, 0, i, cLine);
			if (figure == "Eye") image.draw_line(i, 0, width - 1, i, cLine);
		}
		image.draw_line(0, height - 1, width - 1, height - 1, cLine);
		if (figure == "Eye") image.draw_line(width - 1, 0, width - 1, height - 1, cLine);
	}
	else if (figure == "Diamond")
	{
		const unsigned int middle = width / 2;
		for (unsigned int i = 1; i < middle; i += sidePoints)
		{
			image.draw_line(middle, width - i, middle + i, middle, cLine);
			image.draw_line(middle, i, middle + i, middle, cLine);
			image.draw_line(middle, i, middle - i, middle, cLine);
			image.draw_line(i, middle, middle, middle + i, cLine);
			image.draw_line(0, middle, width - 1, middle, cLine);
		}
	}

	return image;
}

img::EasyImage generate_image(const ini::Configuration &configuration)
{
	std::string type = configuration["General"]["type"].as_string_or_die();
	const unsigned int width = configuration["ImageProperties"]["width"].as_int_or_die();
	const unsigned int height = configuration["ImageProperties"]["height"].as_int_or_die();
	if (type == "IntroColorRectangle")
	{
		return ColorRectangle(width, height);
	}
	if (type == "IntroBlocks")
	{
		const std::vector<double> cWhite = configuration["BlockProperties"]["colorWhite"].as_double_tuple_or_die();
		const std::vector<double> cBlack = configuration["BlockProperties"]["colorBlack"].as_double_tuple_or_die();
		const unsigned int nrXBlocks = configuration["BlockProperties"]["nrXBlocks"].as_int_or_die();
		const unsigned int nrYBlocks = configuration["BlockProperties"]["nrYBlocks"].as_int_or_die();
		const bool invC = configuration["BlockProperties"]["invertColors"].as_bool_or_die();
		return Blocks(width, height, cWhite, cBlack, nrXBlocks, nrYBlocks, invC);
	}
	if (type == "IntroLines")
	{
		const std::string figure = configuration["LineProperties"]["figure"].as_string_or_die();
		const std::vector<double> bgColor = configuration["LineProperties"]["bgColor"].as_double_tuple_or_die();
		const std::vector<double> lineColor = configuration["LineProperties"]["lineColor"].as_double_tuple_or_die();
		const unsigned int nrLines = configuration["LineProperties"]["nrLines"].as_int_or_die();
		return Lines(width, height, figure, bgColor, lineColor, nrLines);
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
