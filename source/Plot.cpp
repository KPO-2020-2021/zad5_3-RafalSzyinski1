/**
 * @file Plot.cpp
 * Definition of Plot class
 */
#include "Plot.h"
#include "Constants.h"

/**
 * Add figure to drawing objects
 * @param fig figure to draw
 */
void Plot::add(const Drawable& fig)
{
    auto obj = fig.getDrawString();
    for (auto& i : obj)
        objects.push_back(" set object " + std::to_string(objects.size() + 1) + " " + i);
}


/** Draw every object which add with add method and clear them */
void Plot::drawAndClear()
{
    sendcommand("set xyplane at 0");
    sendcommand("set xrange [" + std::to_string(-MAP_SIZE) + ":"  + std::to_string(MAP_SIZE) + "]");
    sendcommand("set yrange [" + std::to_string(-MAP_SIZE) + ":"  + std::to_string(MAP_SIZE) + "]");
    sendcommand("set zrange [" + std::to_string(-MAP_SIZE) + ":"  + std::to_string(MAP_SIZE) + "]");
    sendcommand("unset border");
    sendcommand("unset tics");

    // Flush object
    objects.push_back(" set object " + std::to_string(objects.size() + 1) + " polygon from 0,0,0 to 0,0,0 ");

    for (auto& i : objects)
        sendcommand(i);

    sendcommand(R"(splot 0 lc "#00000000" notitle)");
    objects.clear();
}

