/**
 * @file Drone.cpp
 * Definition of Drone class
 */
#include "Drone.h"
#include "Constants.h"
#include "VectorAction.h"
#include "Transform.h"


/** @return reference to propeller */
Propeller& Drone::getPropellerAt(int index)
{
    return propellers.at(index);
}

/**
 * Parametric constructor \n Use VectorAction.h
 * @param x position in x axis
 * @param y position in y axis
 */
Drone::Drone(double x, double y) : Cuboid({DRONE_WIDTH, 0, 0}, {0, DRONE_LENGTH, 0}, {0, 0, DRONE_HEIGHT})
{
    using namespace VectorAction;
    sP = {x - getCenterOfMass().at(0), y - getCenterOfMass().at(1), 0};
    std::vector<double> heightOfPropeller = {0, 0, PROPELLER_HEIGHT / 2.0};
    propellers.emplace_back(sP + Figure::z + heightOfPropeller);
    propellers.emplace_back(sP + Figure::x + Figure::z + heightOfPropeller);
    propellers.emplace_back(sP + Figure::y + Figure::z + heightOfPropeller);
    propellers.emplace_back(sP + Figure::x + Figure::y + Figure::z + heightOfPropeller);
}

/** Use Transform method to rotate drone */
void Drone::spin(double angle)
{
    using VectorAction::operator+;
    Transform::rotateAroundCenterOfMass(*this, angle);
    for (auto& i : propellers)
        i.rotateAround(angle, sP + getCenterOfMass());
}

/** Use Transform method to move drone */
void Drone::move(const std::vector<double>& vec)
{
    Transform::translate(*this, vec);
    for(auto& i : propellers)
        i.move(vec);
}

/** @return const vector of direction */
const std::vector<double>& Drone::getDirection() const
{
    return y;
}

/** Override method to change color of object */
void Drone::changeColor(std::string _color)
{
    for (auto& i : propellers)
        i.changeColor(_color);
    Drawable::changeColor(_color);
}

/**
 * Override method to draw object in gnuplot \n Use VectorAction.h
 * @return list of object to draw
 */
std::list<std::string> Drone::getDrawString() const
{
    using namespace VectorAction;
    std::list<std::string> ret;
    for (auto& i : propellers)
    {
        auto pString = i.getDrawString();
        ret.insert(ret.end(), pString.begin(), pString.end());
    }
    auto cString = Cuboid::getDrawString();
    ret.insert(ret.end(), cString.begin(), cString.end());

    auto cM = getCenterOfMass();
    ret.push_back(getTriangleString(sP + y, z, cM));
    ret.push_back(getTriangleString(sP + y, x, cM));
    ret.push_back(getTriangleString(sP + y + x + z, -x, cM - (x + z)));
    ret.push_back(getTriangleString(sP + y + x, z, cM - x));
    return ret;
}