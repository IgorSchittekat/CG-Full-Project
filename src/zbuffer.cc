#include "zbuffer.hh"


#include <limits>
#include <assert.h>
#include <math.h>
#include <algorithm>



ZBuffer::ZBuffer(const int width, const int height) {
	buffer = new std::vector<std::vector<double>* >;
	for (int j = 0; j < width; j++) {
		std::vector<double>* hor_pixels = new std::vector<double>;
		for (int i = 0; i < height; i++) {
			hor_pixels->push_back(std::numeric_limits<double>::infinity());
		}
		buffer->push_back(hor_pixels);
	}
}

ZBuffer::~ZBuffer() {
	for (int i = buffer->size(); i > 0; i--) {
		delete (*buffer)[i-1];
	}
	delete buffer;
}

double ZBuffer::getBuffer(const unsigned int x, const unsigned int y) const {
	return (*(*buffer)[x])[y];
}

void ZBuffer::setBuffer(const unsigned int x, const unsigned int y, double value) {
	(*(*buffer)[x])[y] = value;
}

void ZBuffer::draw_zbuf_line(img::EasyImage& img, unsigned int x0, unsigned int y0,
	const double z0, unsigned int x1, unsigned int y1, const double z1, const img::Color& color) {
	assert(x0 < img.get_width() && y0 < img.get_height());
	assert(x1 < img.get_width() && y1 < img.get_height());
	Vector3D point0 = Vector3D::point(x0, y0, z0);
	Vector3D point1 = Vector3D::point(x1, y1, z1);
	if (x0 == x1) {
		//special case for x0 == x1
		unsigned int ymin = std::min(y0, y1);
		unsigned int ymax = std::max(y0, y1);
		double zmin = (ymin == y0) ? z0 : z1;
		double zmax = (ymax == y0) ? z0 : z1;
		for (unsigned int i = std::min(y0, y1); i <= std::max(y0, y1); i++) {
			int x = x0;
			int y = i;
			double  z = zmin + (zmax - zmin)*((i - ymin) / (ymax - ymin));
			if (getBuffer(x, y) > 1/z) {
				setBuffer(x, y, 1/z);
				(img)(x, y) = color;
			}
		}
	}		
		
	else if (y0 == y1) {
		//special case for y0 == y1
		unsigned int xmin = std::min(x0, x1);
		unsigned int xmax = std::max(x0, x1);
		double zmin = (xmin == x0) ? z0 : z1;
		double zmax = (xmax == x0) ? z0 : z1;

		for (unsigned int i = std::min(x0, x1); i <= std::max(x0, x1); i++) {
			int x = i;
			int y = y0;
			double z = zmin + (zmax - zmin)*((i - xmin) / (xmax - xmin));
			if (getBuffer(x, y) > 1/z) {
				setBuffer(x, y, 1/z);
				(img)(x, y) = color;
			}
		}
	}
	else {
		if (x0 > x1) {
			//flip points if x1>x0: we want x0 to have the lowest value
			std::swap(x0, x1);
			std::swap(y0, y1);
		}
		double m = ((double) y1 - (double) y0) / ((double) x1 - (double) x0);
		if (-1.0 <= m && m <= 1.0) {
			for (unsigned int i = 0; i <= (x1 - x0); i++) {
				int x = x0 + i;
				int y = round(y0 + m * i);
				double z = z0 + i*(z1 - z0) / (x1 - x0);
				if (getBuffer(x, y) > 1/z) {
					setBuffer(x, y, 1/z);
					(img)(x, y) = color;
				}
			}
		}
		else if (m > 1.0) {
			for (unsigned int i = 0; i <= (y1 - y0); i++) {
				int x = round(x0 + (i / m));
				int y = y0 + i;
				double z = z0 + i*(z1 - z0) / (y1 - y0);
				if (getBuffer(x, y) > 1/z) {
					setBuffer(x, y, 1/z);
					(img)(x, y) = color;
				}
			}
		}
		else if (m < -1.0) {
			for (unsigned int i = 0; i <= (y0 - y1); i++) {
				int x = round(x0 - (i / m));
				int y = y0 - i;
				double z = z0 + i*(z1 - z0) / (y0 - y1);
				if (getBuffer(x, y) > 1/z) {
					setBuffer(x, y, 1/z);
					(img)(x, y) = color;
				}
			}
		}
	}
}