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
};

typedef std::vector<Light> Lights3D;
typedef Vector3D ldVector;
typedef Vector3D location;

Light calculateLight(const std::string& lightname, const ini::Configuration &configuration);
LightColor ambient(Lights3D& lights, LightColor ambientReflection);


#endif /* LIGHT_INCLUDED */