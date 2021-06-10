/**
 * @file Transform.h
 * Static class Transform which transform Figure class
 */
#ifndef ROTATION3D_TRANSFORM_H
#define ROTATION3D_TRANSFORM_H

#include "Figure.h"

class Transform
{
public:
    static void rotateAroundPoint(Figure& figure, double angle, const std::vector<double>& point = {0, 0 , 0});
    static void rotateAroundCenterOfMass(Figure& figure, double angle);
    static void scaleToCenterOfMass(Figure& figure, double scale);
    static void translate(Figure& figure, const std::vector<double>& vec);
};


#endif //ROTATION3D_TRANSFORM_H
