#include "lSystem.hh"
#include "wireframe.hh"
#include "ini_configuration.hh"
#include "figure3D.hh"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <assert.h>

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
	else if (type == "Wireframe" || type == "ZBufferedWireframe" || type == "ZBuffering") {
		const unsigned int size = configuration["General"]["size"].as_int_or_die();
		const std::vector<double> bgColor = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
		const unsigned int nrFigures = configuration["General"]["nrFigures"].as_int_or_die();
		const std::vector<double> eye = configuration["General"]["eye"].as_double_tuple_or_die();
		Figures3D figures;
		for (unsigned int i = 0; i < nrFigures; i++) {
			const std::string figureName = "Figure" + std::to_string(i);
			Figures3D newFigures = calculateFigure(figureName, configuration);
			figures.insert(figures.end(), newFigures.begin(), newFigures.end());
		}
		img::Color c(bgColor[0] * 255, bgColor[1] * 255, bgColor[2] * 255);
		Vector3D eyePoint = Vector3D::point(eye[0], eye[1], eye[2]);
		Matrix M = eyePointTrans(eyePoint);
		applyTransformation(figures, M);
		if (type == "ZBuffering") {
			return drawFigures(figures, size, c);
		}
		Lines2D lines = doProjection(figures);
		if (type == "ZBufferedWireframe") {
			return draw2DLines(lines, size, c, true);
		}
		else if (type == "LightedZBuffering") {
			const unsigned int size = configuration["General"]["size"].as_int_or_die();
			const std::vector<double> bgColor = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
			const std::vector<double> eye = configuration["General"]["eye"].as_double_tuple_or_die();
			const unsigned int nrFigures = configuration["General"]["nrFigures"].as_int_or_die();
			for (unsigned int i = 0; i < nrFigures; i++) {
				const std::string figureName = "Figure" + std::to_string(i);
				Figures3D newFigures = calculateFigure(figureName, configuration);
				figures.insert(figures.end(), newFigures.begin(), newFigures.end());
			}
			//const unsigned int nrLights;
		}
		else 
			return draw2DLines(lines, size, c);
	}
	return img::EasyImage();
}

int main(int argc, char const* argv[]) {
	int retVal = 0;
	try {
		for(int i = 1; i < argc; ++i) {
			ini::Configuration conf;
			try {
				std::ifstream fin(argv[i]);
				fin >> conf;
				fin.close();
			}
			catch(ini::ParseException& ex) {
				std::cerr << "Error parsing file: " << argv[i] << ": " << ex.what() << std::endl;
				retVal = 1;
				continue;
			}

			img::EasyImage image = generate_image(conf);
			if(image.get_height() > 0 && image.get_width() > 0) {
				std::string fileName(argv[i]);
				std::string::size_type pos = fileName.rfind('.');
				if(pos == std::string::npos) {
					//filename does not contain a '.' --> append a '.bmp' suffix
					fileName += ".bmp";
				}
				else {
					fileName = fileName.substr(0,pos) + ".bmp";
				}
				try {
					std::ofstream f_out(fileName.c_str(),std::ios::trunc | std::ios::out | std::ios::binary);
					f_out << image;

				}
				catch(std::exception& ex) {
					std::cerr << "Failed to write image to file: " << ex.what() << std::endl;
					retVal = 1;
				}
			}
			else {
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
