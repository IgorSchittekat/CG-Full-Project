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
	std::vector<double> location = configuration[lightName]["location"].as_double_tuple_or_default({ 0.0, 0.0, 0.0 });
	light.direction = Vector3D::vector(direction[0], direction[1], direction[2]);
	light.location = Vector3D::point(location[0], location[1], location[2]);
	const std::vector<double> eye = configuration["General"]["eye"].as_double_tuple_or_die();
	Vector3D eyePoint = Vector3D::point(eye[0], eye[1], eye[2]);
	Matrix M = eyePointTrans(eyePoint);
	light.direction *= M;
	light.location *= M;
	return light;
}

LightColor ambient(Lights3D& lights, LightColor& ambientReflection) {
	LightColor mixedLights = { 0.0, 0.0, 0.0 };
	for (uint i = 0; i < lights.size(); i++) {
		mixedLights.at(0) += lights.at(i).ambientLight.at(0);
		mixedLights.at(1) += lights.at(i).ambientLight.at(1);
		mixedLights.at(2) += lights.at(i).ambientLight.at(2);
	}
	return { mixedLights[0] * ambientReflection[0], mixedLights[1] * ambientReflection[1], mixedLights[2] * ambientReflection[2] };
}

LightColor diffuse(Lights3D& lights, LightColor& diffuseReflection, Vector3D& n, Vector3D& point) {
	LightColor mixedLights = { 0.0, 0.0, 0.0 };
	for (uint i = 0; i < lights.size(); i++) {
		Vector3D l;
		if (lights.at(i).infinity) {
			l = -(lights.at(i).direction / lights.at(i).direction.length());
		}
		else {
			l = Vector3D::normalise(lights.at(i).location - point);
		}
		LightColor light = { lights[i].diffuseLight[0] * diffuseReflection[0],
			lights[i].diffuseLight[1] * diffuseReflection[1] ,
			lights[i].diffuseLight[2] * diffuseReflection[2] };
		double cosAlpha = n.x * l.x + n.y * l.y + n.z * l.z;
		if (cosAlpha >= 0) {
			mixedLights[0] += light[0] * cosAlpha;
			mixedLights[1] += light[1] * cosAlpha;
			mixedLights[2] += light[2] * cosAlpha;
		}
	}
	return mixedLights;
}

LightColor specular(Lights3D& lights, LightColor& specularReflection, double reflectionCoeff, 
	Vector3D& n, Vector3D& point, Vector3D& eye) {
	Vector3D e = eye - point;
	LightColor mixedLights = { 0.0, 0.0, 0.0 };
	for (uint i = 0; i < lights.size(); i++) {
		Vector3D l;
		if (lights.at(i).infinity) {
			l = -(lights.at(i).direction / lights.at(i).direction.length());
		}
		else {
			l = Vector3D::normalise(lights.at(i).location - point);
		}
		LightColor light = { lights[i].specularLight[0] * specularReflection[0],
			lights[i].specularLight[1] * specularReflection[1] ,
			lights[i].specularLight[2] * specularReflection[2] };
		double cosAlpha = n.x * l.x + n.y * l.y + n.z * l.z;
		Vector3D r = 2 * cosAlpha * n - l;
		double cosBetha = (r.x * e.x + r.y * e.y + r.z * e.z) / (r.length() * e.length());
		if (cosBetha >= 0) {
			mixedLights[0] += light[0] * std::pow(cosBetha, reflectionCoeff);
			mixedLights[1] += light[1] * std::pow(cosBetha, reflectionCoeff);
			mixedLights[2] += light[2] * std::pow(cosBetha, reflectionCoeff);
		}
	}
	return mixedLights;
}
