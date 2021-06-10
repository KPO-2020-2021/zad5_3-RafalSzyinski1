/**
 * @file Drawable.h
 * Base class designed to be able to draw objects
 */
#ifndef ROTATION3D_DRAWABLE_H
#define ROTATION3D_DRAWABLE_H

#include <string>
#include <vector>
#include <list>

class Drawable
{
private:
    std::string color;
    std::string getStyle() const;
protected:
    std::string getParallelogramString(const std::vector<double>& sP, const std::vector<double>& a, const std::vector<double>& b) const;
    std::string getTriangleString(const std::vector<double>& sP, const std::vector<double>& a, const std::vector<double>& b) const;
    std::string getLineString(const std::vector<double>& sP, const std::vector<double>& eP) const;
public:
    Drawable();
    virtual void changeColor(std::string _color);
    virtual std::list<std::string> getDrawString() const = 0;
};

#endif //ROTATION3D_DRAWABLE_H