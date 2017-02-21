#ifndef HELPERS_INCLUDED
#define HELPERS_INCLUDED

#include "easy_image.hh"
#include <vector>
#include <cmath>

namespace help {
	class Color {
		double red;
		double green;
		double blue;
	};

	class Point2D {
	public:
		double x;
		double y;
	};

	class Line2D {
	public:
		Point2D p1;
		Point2D p2;
		img::Color c;
	};

	//int roundToInt(double d);

	img::EasyImage draw2DLines(const std::vector<Line2D> &lines, int size);
	

	

}











#endif /* HELPERS_INCLUDED */