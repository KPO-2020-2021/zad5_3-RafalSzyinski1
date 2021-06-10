/**
 * @file Plot.h
 * Plot class drawing every object
 */
#ifndef ROTATION3D_PLOT_H
#define ROTATION3D_PLOT_H

#include "gplot++.h"
#include "Drawable.h"

#include <list>

class Plot : public Gnuplot
{
private:
    std::list<std::string> objects;
public:
    Plot() = default;
    void add(const Drawable& fig);
    void drawAndClear();
};


#endif //ROTATION3D_PLOT_H
