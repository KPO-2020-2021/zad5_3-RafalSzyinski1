/**
 * @file Drawable.cpp
 * Definition of Drawable class
 */

#include "Drawable.h"
#include "VectorAction.h"

#include <sstream>

/** @return style of object */
std::string Drawable::getStyle() const
{
    return R"( fs border lc "#000000" transparent solid fc rgb ")" + color + "\"";
}

/** Default constructor set color to orange */
Drawable::Drawable() : color("#E65B00")
{
}

/**
 * Draw parallelogram in gnuplot \n Use VectorAction.h
 * @param sP start point
 * @param a first vector starting in sP
 * @param b second vector starting in sP
 * @return string for gnuplot
 */
std::string Drawable::getParallelogramString(const std::vector<double>& sP, const std::vector<double>& a, const std::vector<double>& b) const
{
    using namespace VectorAction;
    std::stringstream ss;
    ss << " polygon from " << sP << " to " << (sP + a) << " to " << (sP + a + b) << " to " << (sP + b) << " to " << sP;
    ss << getStyle();
    return ss.str();
}

/**
 * Draw triangle in gnuplot \n Use VectorAction.h
 * @param sP start point
 * @param a first vector starting in sP
 * @param b second vector starting in sP
 * @return string for gnuplot
 */
std::string Drawable::getTriangleString(const std::vector<double>& sP, const std::vector<double>& a,
                                        const std::vector<double>& b) const
{
    using namespace VectorAction;
    std::stringstream ss;
    ss << " polygon from " << sP << " to " << (sP + a) << " to " << (sP + b) << " to " << sP;
    ss << getStyle();
    return ss.str();
}

/**
 * Draw line in gnuplot \n Use VectorAction.h
 * @param sP start point
 * @param eP end point
 * @return string for gnuplot
 */
std::string Drawable::getLineString(const std::vector<double>& sP, const std::vector<double>& eP) const
{
    using namespace VectorAction;
    std::stringstream ss;
    ss << " polygon from " << sP << " to " << eP << " to " << sP;
    ss << getStyle();
    return ss.str();
}

/**
 * Change object color
 * @param _color in hex format e.g. #FF00FF
 * @throw std::invalid_argument when color is the wrong format
 */
void Drawable::changeColor(std::string _color)
{
    if (_color.size() != 7)
        throw std::invalid_argument("Wrong format of color");
    if (_color[0] != '#')
        throw std::invalid_argument("Wrong format of color");

    for (int i = 1; i < _color.size(); ++i)
        if ((_color[i] > 'F' || _color[i] < 'A') && (_color[i] > '9' || _color[i] < '0'))
            throw std::invalid_argument("Wrong format of color");
    color = _color;
}