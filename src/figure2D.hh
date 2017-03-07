#ifndef FIGURE2D_INCLUDED
#define FIGURE2D_INCLUDED

#include "easy_image.hh"
#include <vector>
#include <cmath>

class Point2D {
public:
	double x;
	double y;
	Point2D& operator=(const Point2D& other);
};

class Line2D {
public:
	Point2D p1;
	Point2D p2;
	img::Color c;
};
typedef std::vector<Line2D> Lines2D;

int roundToInt(double d);

img::EasyImage draw2DLines(const Lines2D &lines, int size, const img::Color& bgc);

#endif /* FIGURE2D_INCLUDED */