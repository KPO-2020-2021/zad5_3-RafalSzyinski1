/**
 * @file Transform.cpp
 * Definition of Transform class
 */

#include "Transform.h"
#include "MatrixAction.h"

/**
 * Rotate figure around selected point in Z axis \n Use VectorAction.h and MatrixAction.h
 * @param figure rotated figure
 * @param angle angle of rotation
 * @param point selected point
 */
void Transform::rotateAroundPoint(Figure& figure, double angle, const std::vector<double>& point)
{
    using namespace VectorAction;
    using namespace MatrixAction;
    matrix<double> m({{std::cos(M_PI/180.0 * angle), -std::sin(M_PI/180.0 * angle), 0}, {std::sin(M_PI/180.0 * angle), std::cos(M_PI/180.0 * angle), 0}, {0, 0, 1}});
    figure.sP = (m * (figure.getCenterOfMass() + figure.sP - point)) +  point - figure.getCenterOfMass();
    rotateAroundCenterOfMass(figure, angle);
}

/**
 * Rotate figure around center of mass in Z axis \n Use VectorAction.h and MatrixAction.h
 * @param figure rotated figure
 * @param angle angle of rotation
 */
void Transform::rotateAroundCenterOfMass(Figure& figure, double angle)
{
    using namespace VectorAction;
    using namespace MatrixAction;
    matrix<double> m({{std::cos(M_PI/180.0 * angle), -std::sin(M_PI/180.0 * angle), 0}, {std::sin(M_PI/180.0 * angle), std::cos(M_PI/180.0 * angle), 0}, {0, 0, 1}});

    auto cM = figure.getCenterOfMass();
    figure.x = (m * (figure.x - cM)) + (m * cM);
    figure.y = (m * (figure.y - cM)) + (m * cM);
    figure.z = (m * (figure.z - cM)) + (m * cM);
    figure.sP = (cM - (m * cM)) + figure.sP;
}

/**
 * Scales figure relative to the center \n Use VectorAction.h and MatrixAction.h
 * @param figure scales figure
 * @param scale scale
 */
void Transform::scaleToCenterOfMass(Figure& figure, double scale)
{
    using namespace VectorAction;
    using namespace MatrixAction;
    auto cP = figure.getCenterOfMass() + figure.sP;
    auto cM = figure.getCenterOfMass();
    figure.x = ((figure.x - cM) * scale) + (cM * scale);
    figure.y = ((figure.y - cM) * scale) + (cM * scale);
    figure.z = ((figure.z - cM) * scale) + (cM * scale);
    figure.sP = cP - (cM * scale);
}

/**
 * Translate figure \n Use VectorAction.h
 * @param figure figure
 * @param vec translation vector
 */
void Transform::translate(Figure& figure, const std::vector<double>& vec)
{
    using namespace VectorAction;
    figure.sP = figure.sP + vec;
}