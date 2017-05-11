#ifndef LIGHT_INCLUDED
#define LIGHT_INCLUDED
#pragma once

#include <vector>
#include "vector.hh"
#include "ini_configuration.hh"

typedef std::vector<double> LightColor;

class Light {
public:
	LightColor ambientLight;
	LightColor diffuseLight;
	LightColor specularLight;
	Vector3D direction;
	Vector3D location;
	bool infinity;
};

typedef std::vector<Light> Lights3D;

Light calculateLight(const std::string& lightname, const ini::Configuration &configuration);
LightColor ambient(Lights3D& lights, LightColor& ambientReflection);
LightColor diffuse(Lights3D& lights, LightColor& diffuseReflection, Vector3D& n, Vector3D& point);
LightColor specular(Lights3D& lights, LightColor& specularReflection, double reflectionCoeff,
	Vector3D& n, Vector3D& point, Vector3D& eye);


#endif /* LIGHT_INCLUDED */