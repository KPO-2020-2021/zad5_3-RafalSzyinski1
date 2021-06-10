/**
 * @file Rectangular.h
 * Declaration of Rectangular class
 */

#ifndef ROTATION_RECTANGULAR_H
#define ROTATION_RECTANGULAR_H

#include "MatrixAction.h"


class Rectangular
{
private:
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> Z;
    std::vector<double> StartPoint;

    std::string color;
public:
    Rectangular() = delete;
    Rectangular(std::vector<double>  _x, std::vector<double>  _y, std::vector<double>  _z, std::vector<double> _startPoint = std::vector<double>(), std::string _color = "#FF0000");

    friend bool operator==(const Rectangular& rec1, const Rectangular& rec2);

    Rectangular operator*(const matrix<double>& mat) const;
    Rectangular operator+(const std::vector<double>& vec) const;

    const std::vector<double>& x() const;
    const std::vector<double>& y() const;
    const std::vector<double>& z() const;
    const std::vector<double>& startPoint() const;
    std::vector<double> centerOfMass() const;
    const std::string getColor() const;
};

namespace Transform
{
    Rectangular rotateRectangularAroundCenterZ(const Rectangular& rec, double angle);

};


#endif //ROTATION_RECTANGULAR_H
