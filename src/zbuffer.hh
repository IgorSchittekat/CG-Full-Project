#ifndef ZBUFFER_INCLUDED
#define ZBUFFER_INCLUDED
#pragma once

#include "easy_image.hh"
#include "figure3D.hh"
#include "figure2D.hh"
#include "vector.hh"

class Point2D;

class ZBuffer {
private:
	std::vector<std::vector<double>* >* buffer;
public:
	ZBuffer(const int width, const int height);
	~ZBuffer();
	double getBuffer(const unsigned int x, const unsigned int y) const;
	void setBuffer(const unsigned int x, const unsigned int y, double value);
	void draw_zbuf_line(img::EasyImage& img, const unsigned int x0, const unsigned int y0, 
		const double z0, const unsigned int x1, const unsigned int y1, const double z1, const img::Color& c);
	void draw_zbuf_triag(img::EasyImage&, const Vector3D& A, const Vector3D& B, const Vector3D& C,
		double d, double dx, double dy, img::Color color);
};



#endif //ZBUFFER_INCLUDED