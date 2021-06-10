/**
 * @file Tetrahedron.h
 * Tetrahedron class
 */
#ifndef ROTATION3D_TETRAHEDRON_H
#define ROTATION3D_TETRAHEDRON_H

#include "Figure.h"

class Tetrahedron : public Figure
{
public:
    Tetrahedron() = delete;
    Tetrahedron(std::vector<double> _x, std::vector<double> _y, std::vector<double> _z, std::vector<double> _sP = {0, 0, 0});
    std::list<std::string> getDrawString() const override;

};


#endif //ROTATION3D_TETRAHEDRON_H
