#include "light.hh"
#include "figure3D.hh"
#include <iostream>

Light calculateLight(const std::string& lightName, const ini::Configuration &configuration) {
	Light light;
	LightColor ambientDefault = { 1.0, 1.0, 1.0 };
	LightColor diffuseDefault = { 0.0, 0.0, 0.0 };
	LightColor specularDefault = { 0.0, 0.0, 0.0 };
	light.ambientLight = configuration[lightName]["ambientLight"].as_double_tuple_or_default(ambientDefault);
	light.diffuseLight = configuration[lightName]["diffuseLight"].as_double_tuple_or_default(diffuseDefault);
	light.specularLight = configuration[lightName]["specularLight"].as_double_tuple_or_default(specularDefault);
	light.infinity = configuration[lightName]["infinity"].as_bool_or_default(true);
	std::vector<double> direction = configuration[lightName]["direction"].as_double_tuple_or_default({ 0.0, 0.0, 0.0 });
	light.direction = Vector3D::vector(direction[0], direction[1], direction[2]);
	const std::vector<double> eye = configuration["General"]["eye"].as_double_tuple_or_die();
	Vector3D eyePoint = Vector3D::point(eye[0], eye[1], eye[2]);
	Matrix M = eyePointTrans(eyePoint);
	light.direction *= M;
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

LightColor diffuse(Lights3D& lights, LightColor diffuseReflection, Vector3D& n) {
	Vector3D l;
	LightColor mixedLights = { 0.0, 0.0, 0.0 };
	for (uint i = 0; i < lights.size(); i++) {
		l = -(lights[i].direction / lights[i].direction.length());
		LightColor light = { lights[i].diffuseLight[0] * diffuseReflection[0],
			lights[i].diffuseLight[1] * diffuseReflection[1] ,
			lights[i].diffuseLight[2] * diffuseReflection[2] };
		double cosAngle = n.x * l.x + n.y * l.y + n.z * l.z;
		if (cosAngle > 0) {
			mixedLights[0] += light[0] * cosAngle;
			mixedLights[1] += light[1] * cosAngle;
			mixedLights[2] += light[2] * cosAngle;
		}
	}
	return mixedLights;
}