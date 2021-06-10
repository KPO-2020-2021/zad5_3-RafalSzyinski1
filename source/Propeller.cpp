/**
 * @file Propeller.cpp
 * Definition of Propeller class
 */
#include "Propeller.h"
#include "Constants.h"
#include "VectorAction.h"
#include "Transform.h"

/**
 * Constructor create a propeller form 3 rotated cuboids with specific dimensions \n
 * Use VectorAction.h
 * @param centerPoint
 */
Propeller::Propeller(const std::vector<double>& centerPoint)
{
    using namespace VectorAction;
    auto sP = centerPoint - Cuboid({PROPELLER_WIDTH, 0, 0}, {0, PROPELLER_LENGTH, 0}, {0, 0, PROPELLER_HEIGHT}).getCenterOfMass();
    Cuboid temp({PROPELLER_WIDTH, 0, 0}, {0, PROPELLER_LENGTH, 0}, {0, 0, PROPELLER_HEIGHT}, sP);
    cuboids.push_back(temp);
    Transform::rotateAroundCenterOfMass(temp, 120);
    cuboids.push_back(temp);
    Transform::rotateAroundCenterOfMass(temp, 120);
    cuboids.push_back(temp);
}

/** Use Transform method to rotate propeller */
void Propeller::spin(double angle)
{
    for(auto& i : cuboids)
        Transform::rotateAroundCenterOfMass(i, angle);
}

/** Use Transform method to move propeller */
void Propeller::move(const std::vector<double>& vec)
{
    for (auto& i : cuboids)
        Transform::translate(i, vec);
}

/** Use Transform method to rotate around point propeller */
void Propeller::rotateAround(double angle, const std::vector<double>& vec)
{
    for (auto& i : cuboids)
        Transform::rotateAroundPoint(i, angle, vec);
}

/** Override method to change color of object */
void Propeller::changeColor(std::string _color)
{
    for (auto& i : cuboids)
        i.changeColor(_color);
}

/**
 * Override method to draw object in gnuplot \n Use VectorAction.h
 * @return list of object to draw
 */
std::list<std::string> Propeller::getDrawString() const
{
    std::list<std::string> ret;
    for (auto& i : cuboids)
    {
        auto temp = i.getDrawString();
        ret.insert(ret.end(), temp.begin(), temp.end());
    }
    return ret;
}