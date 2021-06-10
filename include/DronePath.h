/**
 * @file DronePath.h
 * DronePath class
 */
#ifndef ROTATION3D_DRONEPATH_H
#define ROTATION3D_DRONEPATH_H

#include "Constants.h"
#include "Figure.h"

#include <deque>
#include <optional>
#include <stdexcept>
#include <memory>

class Scene;
class AutoDrone;

class DronePath : public Drawable
{
private:
    std::deque<FlyStates> moves;
    const AutoDrone& drone;
    const Scene& scene;

    double height;
protected:
    void fillAscentMoves();
    void fillFlyingMoves(double distance);
    void fillRotateMoves(double angle);
public:
    DronePath(const AutoDrone& _drone, const Scene& _scene);
    FlyStates getNextMove();
    void makeMoves(double angle, double distance);
    std::optional<std::shared_ptr<Figure> > collisionTest() const;
    void overtaking(const Figure& fig);
    void overtaking(const AutoDrone& ad);
    void fillLandingMoves();
    std::list<std::string> getDrawString() const override;
};

#endif //ROTATION3D_DRONEPATH_H
