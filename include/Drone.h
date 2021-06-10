/**
 * @file Drone.h
 * Drone class
 */
#ifndef ROTATION3D_DRONE_H
#define ROTATION3D_DRONE_H

#include "Cuboid.h"
#include "Propeller.h"

class Drone : public Cuboid
{
private:
    std::vector<Propeller> propellers;
protected:
    Propeller& getPropellerAt(int index);
public:
    explicit Drone(double x = 0, double y = 0);
    void spin(double angle);
    void move(const std::vector<double>& vec);
    const std::vector<double>& getDirection() const;
    void changeColor(std::string _color) override;
    std::list<std::string> getDrawString() const override;
};

#endif //ROTATION3D_DRONE_H