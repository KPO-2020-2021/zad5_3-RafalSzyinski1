/**
 * @file Plane.cpp
 * Definition of Plane.cpp class
 */
#include "Plane.h"
#include "VectorAction.h"

/**
 * Parametric constructor
 * @param _x x vector starting in sP
 * @param _y y vector starting in sP
 * @param _z z vector starting in sP
 * @param _sP start point
 */
Plane::Plane(std::vector<double> _x, std::vector<double> _y, std::vector<double> _z, std::vector<double> _sP)
            : Figure(std::move(_x), std::move(_y), std::move(_z), std::move(_sP))
{
}

/**
 * Override method to draw object in gnuplot \n Use VectorAction.h
 * @return list of object to draw
 */
std::list<std::string> Plane::getDrawString() const
{
    using namespace VectorAction;
    std::list<std::string> ret;
    ret.push_back(Drawable::getParallelogramString(sP, x, y));
    ret.push_back(Drawable::getParallelogramString(sP, y, z));
    ret.push_back(Drawable::getTriangleString(sP, x, z));
    auto nP = sP + x + y;
    ret.push_back(Drawable::getTriangleString(nP, -x, z - x));
    ret.push_back(Drawable::getParallelogramString(nP, -y, z - x));
    return ret;
}