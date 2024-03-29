#include "figure2D.hh"
#include <cmath>
#include <algorithm>
#include <limits>



int roundToInt(double d) {
	return (d < 0) ? std::ceil(d - 0.5) : std::floor(d + 0.5);
}


img::EasyImage draw2DLines(const Lines2D &lines, int size, const img::Color& bgc, bool zBuffered) {
	double xMax, xMin, yMax, yMin;
	xMax = yMax = -std::numeric_limits<double>::infinity();
	xMin = yMin = std::numeric_limits<double>::infinity();

	for (Line2D line : lines) {
		xMax = std::max({ xMax, line.p1.x, line.p2.x });
		xMin = std::min({ xMin, line.p1.x, line.p2.x });
		yMax = std::max({ yMax, line.p1.y, line.p2.y });
		yMin = std::min({ yMin, line.p1.y, line.p2.y });
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
	img::EasyImage image((int)imageX, (int)imageY, bgc);
	ZBuffer buffer((int)imageX, (int)imageY);
	for (Line2D line : lines) {
		if (!zBuffered)
			image.draw_line(roundToInt(line.p1.x * d + dx), roundToInt(line.p1.y * d + dy),
				roundToInt(line.p2.x * d + dx), roundToInt(line.p2.y * d + dy), line.c);
		else {
			buffer.draw_zbuf_line(image, roundToInt(line.p1.x * d + dx), roundToInt(line.p1.y * d + dy), 
				line.z1, roundToInt(line.p2.x * d + dx), roundToInt(line.p2.y * d + dy), line.z2, line.c);
		}
	}
	return image;
}

Point2D::Point2D() {
}

Point2D::Point2D(double x, double y) :
	x(x),
	y(y) {
}

Point2D& Point2D::operator=(const Point2D& other) {
	x = other.x;
	y = other.y;
	return *this;
}

