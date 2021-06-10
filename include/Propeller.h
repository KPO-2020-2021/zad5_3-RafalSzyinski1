/**
 * @file Propeller.h
 * Propeller class
 */
#ifndef ROTATION3D_PROPELLER_H
#define ROTATION3D_PROPELLER_H

#include "Cuboid.h"

class Propeller : public Drawable
{
private:
    std::vector<Cuboid> cuboids;
public:
    Propeller() = delete;
    explicit Propeller(const std::vector<double>& centerPoint);
    void spin(double angle);
    void move(const std::vector<double>& vec);
    void rotateAround(double angle, const std::vector<double>& vec);
    void changeColor(std::string _color) override;
    std::list<std::string> getDrawString() const override;
};


#endif //ROTATION3D_PROPELLER_H
