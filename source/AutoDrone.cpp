/**
 * @file AutoDrone.cpp
 * Definition of AutoDrone class
 */
#include "AutoDrone.h"
#include "Constants.h"
#include "VectorAction.h"

/** Spin propellers while drone flying */
void AutoDrone::spinPropellers()
{
    getPropellerAt(0).spin(-PROPELLER_SPEED);
    getPropellerAt(1).spin(PROPELLER_SPEED);
    getPropellerAt(2).spin(-PROPELLER_SPEED);
    getPropellerAt(3).spin(PROPELLER_SPEED);
}

/** Move drone up  */
void AutoDrone::moveUp()
{
    move({0, 0, SPEED});
}

/** Move drone down */
void AutoDrone::moveDown()
{
    move({0, 0, -SPEED});
}

/** Move drone straight */
void AutoDrone::moveStraight()
{
    using namespace VectorAction;

    for (auto& i : dronePath.collisionTest())
    {
        auto dr = dynamic_cast<AutoDrone*>(i.get());
        if (dr && !(dr->isStop()))
        {
            stop = true;
            dronePath.overtaking(*dr);
            return;
        }
        else
        {
            dronePath.overtaking(*i);
        }
    }

    stop = false;
    auto dir = getDirection();
    move(dir/abs(dir) * SPEED);
}

/** Rotate drone right and spin propellers */
void AutoDrone::rotateRight()
{
    getPropellerAt(0).spin(-PROPELLER_SPEED / 2);
    getPropellerAt(3).spin(PROPELLER_SPEED / 2);
    spin(-SPEED);
}

/** Rotate drone left and spin propellers */
void AutoDrone::rotateLeft()
{
    getPropellerAt(1).spin(PROPELLER_SPEED / 2);
    getPropellerAt(2).spin(-PROPELLER_SPEED / 2);
    spin(SPEED);
}

/**
 * Parametric constructor
 * @param x position in x axis
 * @param y position in y axis
 */
AutoDrone::AutoDrone(double x, double y, const Scene& _scene) : Drone(x, y), flying(false), stop(true), dronePath(*this, _scene)
{
}

/** Set drone path to fly */
void AutoDrone::makePath(double angle, double distance)
{
    flying = true;
    dronePath.makeMoves(angle, distance);
}

/** Flying method */
void AutoDrone::fly()
{
    if (flying)
    {
        spinPropellers();
        switch (dronePath.getNextMove())
        {
            case FlyStates::ASCENT:
                moveUp();
                break;
            case FlyStates::FLYING:
                moveStraight();
                break;
            case FlyStates::LANDING:
                moveDown();
                break;
            case FlyStates::ROTATE_LEFT:
                rotateLeft();
                break;
            case FlyStates::ROTATE_RIGHT:
                rotateRight();
                break;
            case FlyStates::STOP_LAND:
                dronePath.fillLandingMoves();
                break;
            case FlyStates::END:
                flying = false;
                break;
        }

    }
}

/** @return true if drone is flying */
bool AutoDrone::isFlying() const
{
    return flying;
}

/** @return true if drone is not moving */
bool AutoDrone::isStop() const
{
    if (!flying)
        return true;
    return stop;
}

/**
 * Override method to draw object in gnuplot
 * @return list of object to draw
 */
std::list<std::string> AutoDrone::getDrawString() const
{
    auto ret = Drone::getDrawString();
    auto path = dronePath.getDrawString();
    ret.insert(ret.end(), path.begin(), path.end());
    return ret;
}