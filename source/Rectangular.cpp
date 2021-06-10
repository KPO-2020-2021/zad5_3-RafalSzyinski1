/**
 * @file Rectangular.cpp
 * Definition of Rectangular class
 */
#include "Rectangular.h"
#include <sstream>
#include <utility>  


/// Single class constructor.
Rectangular::Rectangular(std::vector<double>  _x, std::vector<double>  _y, std::vector<double>  _z, std::vector<double> _startPoint, std::string _color)
                        : X(std::move(_x)), Y(std::move(_y)), Z(std::move(_z)), StartPoint(std::move(_startPoint)), color(std::move(_color))
{
    if (StartPoint.empty())
        StartPoint = std::vector<double>(X.size());
    if (X.size() != Y.size() || X.size() != Z.size() || X.size() != StartPoint.size())
        throw std::invalid_argument("ERROR Rectangular: x.size() != y.size() || x.size() != z.size() || x.size() != startPoint.size()");
}

/// Function comparing two Rectangular. \n  Using operator== from VectorAction.h.
bool operator==(const Rectangular& rec1, const Rectangular& rec2)
{
    if ((rec1.X == rec2.X) && (rec1.Y == rec2.Y) && (rec1.Z == rec2.Z) && (rec1.StartPoint == rec2.StartPoint))
        return true;
    return false;
}

/// Multiplication by matrix. Multiply every vector of Rectangular by mat. \n Using MatrixAction.h.
Rectangular Rectangular::operator*(const matrix<double>& mat) const
{
    using MatrixAction::operator*;
    Rectangular ret(mat * X, mat * Y, mat * Z, mat * StartPoint);
    return ret;
}

/// Translation by vector. Adding vector to starting point of Rectangular. \n Using VectorAction.h
Rectangular Rectangular::operator+(const std::vector<double>& vec) const
{
    using VectorAction::operator+;
    Rectangular ret(X, Y, Z, StartPoint + vec);
    return ret;
}

/// @return const vector x.
const std::vector<double>& Rectangular::x() const
{
    return X;
}

/// @return const vector y.
const std::vector<double>& Rectangular::y() const
{
    return Y;
}

/// @return const vector z.
const std::vector<double>& Rectangular::z() const
{
    return Z;
}

/// @return const vector startPoint.
const std::vector<double>& Rectangular::startPoint() const
{
    return StartPoint;
}

/// @return vector with center of mass
std::vector<double> Rectangular::centerOfMass() const
{
    using VectorAction::operator*;
    using VectorAction::operator+;
    return (X + Y + Z) * (1.0/2.0);
}

const std::string Rectangular::getColor() const
{
    return color;
}

/// Transform function helping to rotate Rectangular around center of mass in Z axis
Rectangular Transform::rotateRectangularAroundCenterZ(const Rectangular& rec, double angle)
{
    using namespace VectorAction;
    using namespace MatrixAction;
    matrix<double> m({{std::cos(M_PI/180 * angle), -std::sin(M_PI/180 * angle), 0}, {std::sin(M_PI/180 * angle), std::cos(M_PI/180 * angle), 0}, {0, 0, 1}});
    Rectangular temp(rec.centerOfMass() - rec.x(), rec.centerOfMass() - rec.y(), rec.centerOfMass() - rec.z(),  -rec.centerOfMass());
    temp = temp * m;
    return Rectangular(temp.x() + temp.startPoint() , temp.y() + temp.startPoint(), temp.z() + temp.startPoint(), (rec.centerOfMass() - temp.startPoint()) + rec.startPoint());
}

