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
			double z;
			if (y0 == y1)
				z = zmin + i * (zmax - zmin);
			else
				z = zmin + (zmax - zmin)*((i - ymin) / (ymax - ymin));
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

void intersection(double& xI1, double& xI2, double yI, double xP, double yP, double xQ, double yQ) {
	if (yP < yQ) {
		std::swap(yP, yQ);
		std::swap(xP, xQ);
	}
	if ((yI - yP) * (yI - yQ) <= 0 && yP != yQ) {
		xI1 = xI2 = xQ + (xP - xQ) * ((yI - yQ) / (yP - yQ));
	}
}

void ZBuffer::draw_zbuf_triag(img::EasyImage& img, const Vector3D& A, const Vector3D& B, const Vector3D& C,
	double d, double dx, double dy, LightColor ambientReflection, LightColor diffuseReflection,
	LightColor specularReflection, double reflectionCoeff, Lights3D& lights) {
	Point2D pA((d * A.x) / (-A.z) + dx, (d * A.y) / (-A.z) + dy);
	Point2D pB((d * B.x) / (-B.z) + dx, (d * B.y) / (-B.z) + dy);
	Point2D pC((d * C.x) / (-C.z) + dx, (d * C.y) / (-C.z) + dy);
	double xG = (pA.x + pB.x + pC.x) / 3;
	double yG = (pA.y + pB.y + pC.y) / 3;
	double zGInv = (1 / (3 * A.z)) + (1 / (3 * B.z)) + (1 / (3 * C.z));
	Vector3D u = B - A;
	Vector3D v = C - A;
	Vector3D w = Vector3D::vector((u.y * v.z) - (u.z * v.y), (u.z * v.x) - (u.x * v.z), (u.x * v.y) - (u.y * v.x));
	double k = (w.x * A.x) + (w.y * A.y) + (w.z * A.z);
	double dzdx = w.x / (-d * k);
	double dzdy = w.y / (-d * k);
	
	Vector3D n = Vector3D::normalise(w);
	
	LightColor ambientColor = ambient(lights, ambientReflection);
	LightColor diffuseColor = diffuse(lights, diffuseReflection, n);

	LightColor colorVec = { ambientColor.at(0) + diffuseColor.at(0),
		ambientColor.at(1) + diffuseColor.at(1),
		ambientColor.at(2) + diffuseColor.at(2) };
	img::Color color(colorVec[0] * 255, colorVec[1] * 255, colorVec[2] * 255);


	int yMin = roundToInt(std::min({ pA.y, pB.y, pC.y }) + 0.5);
	int yMax = roundToInt(std::max({ pA.y, pB.y, pC.y }) - 0.5);
	for (int y = yMin; y <= yMax; y++) {
		double xLab, xLac, xLbc, xRab, xRac, xRbc;
		xLab = xLac = xLbc = std::numeric_limits<double>::infinity();
		xRab = xRac = xRbc = -std::numeric_limits<double>::infinity();
		intersection(xLab, xRab, (double)y, pA.x, pA.y, pB.x, pB.y);
		intersection(xLac, xRac, (double)y, pA.x, pA.y, pC.x, pC.y);
		intersection(xLbc, xRbc, (double)y, pB.x, pB.y, pC.x, pC.y);
		int xL = roundToInt(std::min({ xLab, xLac, xLbc }) + 0.5);
		int xR = roundToInt(std::max({ xRab, xRac, xRbc }) - 0.5);
		for (int x = xL; x <= xR; x++) {
			double zInv = 1.0001 * zGInv + ((double)x - xG) * dzdx + ((double)y - yG) * dzdy;
			if (zInv < getBuffer(x, y)) {
				(img)(x, y) = color;
				setBuffer(x, y, zInv);
			}
		}
	}
}