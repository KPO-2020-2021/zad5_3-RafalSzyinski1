/**
 * @file Plane.h
 * Plane class
 */
#ifndef ROTATION3D_PLANE_H
#define ROTATION3D_PLANE_H

#include "Figure.h"

class Plane : public Figure
{
public:
    Plane() = delete;
    Plane(std::vector<double> _x, std::vector<double> _y, std::vector<double> _z, std::vector<double> _sP = {0, 0, 0});
    std::list<std::string> getDrawString() const override;
};


#endif //ROTATION3D_PLANE_H
