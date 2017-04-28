#ifndef FIGURE2D_INCLUDED
#define FIGURE2D_INCLUDED
#pragma once

#include "easy_image.hh"
#include "zbuffer.hh"
#include <vector>
#include <cmath>

class Point2D {
public:
	Point2D();
	Point2D(double x, double y);
	double x;
	double y;
	Point2D& operator=(const Point2D& other);
};

class Line2D {
public:
	Point2D p1;
	Point2D p2;
	img::Color c;
	double z1;
	double z2;
};
typedef std::vector<Line2D> Lines2D;

int roundToInt(double d);

img::EasyImage draw2DLines(const Lines2D &lines, int size, const img::Color& bgc, bool zBuffered = false);

#endif /* FIGURE2D_INCLUDED */