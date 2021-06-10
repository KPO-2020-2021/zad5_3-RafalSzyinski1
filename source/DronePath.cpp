/**
 * @file DronePath.cpp
 * Definition of DronePath class
 */
#include "DronePath.h"
#include "AutoDrone.h"
#include "MatrixAction.h"
#include "Scene.h"


/**
 * Parametric constructor
 * @param _drone owner
 * @param _scene scene with all objects
 */
DronePath::DronePath(const AutoDrone& _drone, const Scene& _scene) : drone(_drone), scene(_scene), height(DRONE_HEIGHT / 2)
{
}

/**
 * Use in AutoDrone
 * @return move which AutoDrone have to make
 */
FlyStates DronePath::getNextMove()
{
    if (moves.empty())
        return FlyStates::NONE;
    auto ret = moves.front();
    moves.pop_front();
    return ret;
}

/**
 * Create path for drone
 * @param angle angle of fly
 * @param distance distance of fly
 */
void DronePath::makeMoves(double angle, double distance)
{
    fillAscentMoves();
    fillRotateMoves(angle);
    fillFlyingMoves(distance);
    moves.push_back(FlyStates::STOP_LAND);
}

/** Fill ascent moves */
void DronePath::fillAscentMoves()
{
    double times = (MAX_ALTITUDE - height) / SPEED;
    for (int i = 0; i < times; ++i)
    {
        moves.push_back(FlyStates::ASCENT);
        height += SPEED;
    }
}

/** Fill landing moves */
void DronePath::fillLandingMoves()
{
    // TODO poprawic
    double times = (height - DRONE_HEIGHT / 2) / SPEED;
    for (int i = 0; i < times; ++i)
    {
        moves.push_back(FlyStates::LANDING);
        height -= SPEED;
    }
    moves.push_back(FlyStates::END);
}

/** Fill flying straight moves */
void DronePath::fillFlyingMoves(double distance)
{
    double times = (distance) / SPEED;
    for (int i = 0; i < times; ++i)
        moves.push_back(FlyStates::FLYING);
}

/** Fill rotate moves */
void DronePath::fillRotateMoves(double angle)
{
    angle = angle - (360 *  (int)(angle/360));
    if (angle < 0)
        angle = 360 + angle;
    if (angle > 180)
    {
        angle = 360 - angle;
        double times = angle / SPEED;
        for (int i = 0; i < times; ++i)
            moves.push_back(FlyStates::ROTATE_LEFT);
    }
    else
    {
        double times = angle / SPEED;
        for (int i = 0; i < times; ++i)
            moves.push_back(FlyStates::ROTATE_RIGHT);
    }
}

#include <iostream>

/**
 * Override method to draw object in gnuplot \n Use VectorAction.h and MatrixAction.h
 * @return list of object to draw
 */
std::list<std::string> DronePath::getDrawString() const
{
    using namespace MatrixAction;
    using namespace VectorAction;
    auto pos = drone.getPosition();
    auto dir = drone.getDirection();
    std::list<std::string> ret;
    matrix<double> left({
        {std::cos(M_PI/180.0 * SPEED),-std::sin(M_PI/180.0 * SPEED), 0},
         {std::sin(M_PI/180.0 * SPEED), std::cos(M_PI/180.0 * SPEED), 0},
         {0, 0, 1}});

    matrix<double> right({
         {std::cos(M_PI/180.0 * -SPEED),-std::sin(M_PI/180.0 * -SPEED), 0},
         {std::sin(M_PI/180.0 * -SPEED), std::cos(M_PI/180.0 * -SPEED), 0},
         {0, 0, 1}});

    double count = 0;
    FlyStates countingState = FlyStates::NONE;
    for (auto& i : moves)
    {
       if (i == countingState)
           ++count;
       else
       {
           switch (countingState)
           {
               case FlyStates::ASCENT:
                   ret.push_back(getLineString(pos, pos + std::vector<double>({0, 0, SPEED * count})));
                   pos = pos + std::vector<double>({0, 0, SPEED * count});
                   break;
               case FlyStates::FLYING:
                   ret.push_back(getLineString(pos, pos + (dir/abs(dir) * SPEED * count)));
                   pos = pos + (dir/abs(dir) * SPEED * count);
                   break;
               case FlyStates::LANDING:
                   ret.push_back(getLineString(pos,  pos - std::vector<double>({0, 0, SPEED * count})));
                   pos = pos - std::vector<double>({0, 0, SPEED * count});
                   break;
               case FlyStates::ROTATE_LEFT:
                   for (int j = 0; j < count; ++j)
                        dir = left * dir;
                   break;
               case FlyStates::ROTATE_RIGHT:
                   for (int j = 0; j < count; ++j)
                       dir = right * dir;
                   break;
           }
           count = 1;
           countingState = i;
       }
    }
    return ret;
}

/**
 * Overtaking method
 * Drone will be flight above object
 * @param fig Figure on collision
 */
void DronePath::overtaking(const Figure& fig)
{
    double figureHeight = (fig.getPosition().at(2) + fig.getCenterOfMass().at(2)) + (DRONE_HEIGHT * 2);
    if (figureHeight > height)
    {
        double times = (figureHeight - height) / SPEED;
        for (int i = 0; i < times; ++i)
            moves.push_front(FlyStates::ASCENT);
        height = figureHeight;
    }
}

/**
 * Collision test with all figure in Scene
 * @return std::optional with object on drone way
 */
std::optional<std::shared_ptr<Figure> > DronePath::collisionTest() const
{
    using namespace VectorAction;
    auto dronePos = drone.getPosition();
    dronePos.at(2) = 0;
    auto droneRadius = drone.getCenterOfMass();
    droneRadius.at(2) = 0;
    double radius = abs(droneRadius) * 1.5;
    for (auto& i : scene.objects)
    {
        auto figureRadius = i->getCenterOfMass();
        auto figurePos = i->getPosition();
        figureRadius.at(2) = 0;
        figurePos.at(2) = 0;

        figureRadius.at(0) = std::abs(figureRadius.at(0)) + radius + PROPELLER_LENGTH;
        figureRadius.at(1) = std::abs(figureRadius.at(1)) + radius + PROPELLER_LENGTH;

        auto distance = dronePos - figurePos;

        double figureHeight = (i->getPosition().at(2) + i->getCenterOfMass().at(2)) + DRONE_HEIGHT;

        if (figureHeight >= height)
        {
            if (std::abs(distance.at(0)) < figureRadius.at(0) && std::abs(distance.at(1)) < figureRadius.at(1))
                return std::optional<std::shared_ptr<Figure> >(i);
        }

    }
    return {};
}

/** @return height on which drone have to land or -1 if cannot land */
double DronePath::getLandingHeight() const
{

    return 0;
}
