/**
 * @file Cuboid.h
 * Cuboid class
 */
#ifndef ROTATION3D_CUBOID_H
#define ROTATION3D_CUBOID_H

#include "Figure.h"

class Cuboid : public Figure
{
public:
    Cuboid() = delete;
    Cuboid(std::vector<double> _x, std::vector<double> _y, std::vector<double> _z, std::vector<double> _sP = {0, 0, 0});
    std::list<std::string> getDrawString() const override;
};


#endif //ROTATION3D_CUBOID_H
