#ifndef WIREFRAME_INCLUDED
#define WIREFRAME_INCLUDED
#pragma once

#include "ini_configuration.hh"
#include "figure3D.hh"
#include "figure2D.hh"

Figures3D calculateFigure(const std::string& figureName, const ini::Configuration &configuration);




#endif //WIREFRAME_INCLUDED
