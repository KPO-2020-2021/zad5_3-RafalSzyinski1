/**
 * @file Pyramid.h
 * Pyramid class
 */
#ifndef ROTATION3D_PYRAMID_H
#define ROTATION3D_PYRAMID_H

#include "Figure.h"

class Pyramid : public Figure
{
public:
    Pyramid() = delete;
    Pyramid(std::vector<double> _x, std::vector<double> _y, std::vector<double> _z, std::vector<double> _sP = {0, 0, 0});
    std::list<std::string> getDrawString() const override;
};


#endif //ROTATION3D_PYRAMID_H
