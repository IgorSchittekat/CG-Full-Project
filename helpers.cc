#include "helpers.hh"
#include <cmath>
#include <algorithm>



int roundToInt(double d) {
	return (d < 0) ? std::ceil(d - 0.5) : std::floor(d + 0.5);
}


img::EasyImage draw2DLines(const Lines2D &lines, int size, const img::Color& bgc) {

	double xMax = 0, xMin = 0, yMax = 0, yMin = 0;
	for (Line2D line : lines) {
		xMax = std::max(std::max(xMax, line.p1.x), line.p2.x);
		xMin = std::min(std::min(xMin, line.p1.x), line.p2.x);
		yMax = std::max(std::max(yMax, line.p1.y), line.p2.y);
		yMin = std::min(std::min(yMin, line.p1.y), line.p2.y);
	}
	const double xRange = xMax - xMin;
	const double yRange = yMax - yMin;
	const double imageX = size * (xRange / std::max(xRange, yRange));
	const double imageY = size * (yRange / std::max(xRange, yRange));

	const double d = 0.95 * (imageX / xRange);

	const double DCx = d * (xMin + xMax) / 2;
	const double DCy = d * (yMin + yMax) / 2;

	const double dx = imageX / 2 - DCx;
	const double dy = imageY / 2 - DCy;
	
	img::EasyImage image(roundToInt(imageX+1), roundToInt(imageY+1), bgc);

	for (Line2D line : lines) {
		image.draw_line(roundToInt(line.p1.x * d + dx), roundToInt(line.p1.y * d + dy),
			roundToInt(line.p2.x * d + dx), roundToInt(line.p2.y * d + dy), line.c);
	}
	return image;
}

Point2D& Point2D::operator=(const Point2D& other) {
	x = other.x;
	y = other.y;
	return *this;
}

