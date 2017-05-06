#include "light.hh"
#include <iostream>

Light calculateLight(const std::string& lightName, const ini::Configuration &configuration) {
	Light light;
	LightColor ambientDefault = { 1.0, 1.0, 1.0 };
	LightColor diffuseDefault = { 0.0, 0.0, 0.0 };
	LightColor specularDefault = { 0.0, 0.0, 0.0 };
	light.ambientLight = configuration[lightName]["ambientLight"].as_double_tuple_or_default(ambientDefault);
	light.diffuseLight = configuration[lightName]["diffuseLight"].as_double_tuple_or_default(diffuseDefault);
	light.specularLight = configuration[lightName]["specularLight"].as_double_tuple_or_default(specularDefault);
	return light;
}

LightColor ambient(Lights3D& lights, LightColor ambientReflection) {
	LightColor mixedLights = { 0.0, 0.0, 0.0 };
	for (uint i = 0; i < lights.size(); i++) {
		mixedLights[0] += lights[i].ambientLight[0];
		mixedLights[1] += lights[i].ambientLight[1];
		mixedLights[2] += lights[i].ambientLight[2];
	}
	return { mixedLights[0] * ambientReflection[0], mixedLights[1] * ambientReflection[1], mixedLights[2] * ambientReflection[2] };
}