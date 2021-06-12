/**
 * @file AutoDrone.h
 * AutoDrone class
 */
#ifndef ROTATION3D_AUTODRONE_H
#define ROTATION3D_AUTODRONE_H

#include "Drone.h"
#include "DronePath.h"

class AutoDrone : public Drone
{
private:
    DronePath dronePath;
    bool flying;
    bool stop;
protected:
    void spinPropellers();
    void moveUp();
    void moveDown();
    void moveStraight();
    void rotateRight();
    void rotateLeft();
public:
    AutoDrone() = delete;
    AutoDrone(double x, double y, const Scene& _scene);
    void makePath(double angle, double distance);
    void fly();
    bool isFlying() const;
    bool isStop() const;
    std::list<std::string> getDrawString() const override;
};


#endif //ROTATION3D_AUTODRONE_H
