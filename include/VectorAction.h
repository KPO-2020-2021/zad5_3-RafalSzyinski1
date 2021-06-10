/**
 * @file VectorAction.h
 * Overload operators for std::vector
 */
#ifndef ROTATION2D_VECTORACTION_H
#define ROTATION2D_VECTORACTION_H


#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "Constants.h"

/**
 * Function comparing to 10 decimal places each value of vectors
 * @return True if values in vectors are close to equal
 */
template<typename T>
bool operator==(const std::vector<T>& vec1, const std::vector<T>& vec2)
{
    if (vec1.size() != vec2.size())
        return false;

    const int DEC_PLACE = 10;
    for (int i = 0; i < vec1.size(); ++i)
        if (std::abs(vec1.at(i) - vec2.at(i)) > EPSILON)
            return false;
    return true;
}

namespace VectorAction
{
    /**
     * Multiplication vector by a scalar
     */
    template<typename T>
    std::vector<T> operator*(const std::vector<T>& vec, const T& sc);

    /**
     * Scalar product of 2 vectors
     * @throw std::out_of_range vec1.size() != vec2.size()
     */
    template<typename T>
    double operator*(const std::vector<T>& vec1, const std::vector<T>& vec2);

    /**
     * Division by scalar
     * @throw std::invalid_argument when sc is equal to 0
     */
    template<typename T>
    std::vector<T> operator/(const std::vector<T>& vec, const T& sc);

    /**
     * Sum operator of 2 vectors
     * @throw std::out_of_range vec1.size() != vec2.size()
     */
    template<typename T>
    std::vector<T> operator+(const std::vector<T>& vec1, const std::vector<T>& vec2);

    /**
     * Difference operator of 2 vectors
     * @throw std::out_of_range vec1.size() != vec2.size()
     */
    template<typename T>
    std::vector<T> operator-(const std::vector<T>& vec1, const std::vector<T>& vec2);

    /**
     * Negation operator
     */
    template<typename T>
    std::vector<T> operator-(const std::vector<T>& vec);

    /**
     * Absolute value of the vector
     */
    template<typename T>
    double abs(const std::vector<T>& vec);

    /**
     * Out stream operator to write on std::out and to file
     */
    template<typename T>
    std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec);
}   //VectorAction


template<typename T>
std::vector<T> VectorAction::operator*(const std::vector<T>& vec, const T& sc)
{
    std::vector<T> ret;
    for (auto& i : vec)
        ret.push_back(i * sc);
    return ret;
}

template<typename T>
double VectorAction::operator*(const std::vector<T>& vec1, const std::vector<T>& vec2)
{
    if (vec1.size() != vec2.size())
        throw std::out_of_range("Vector ERROR ScalarProduct: vec1.size() != vec2.size()");
    double ret = 0;

    for (int i = 0; i < vec1.size(); ++i)
        ret += vec1[i] * vec2[i];

    return ret;
}

template<typename T>
std::vector<T> VectorAction::operator/(const std::vector<T>& vec, const T& sc)
{
    if (sc == 0)
        throw std::invalid_argument("sc == 0");

    std::vector<T> ret;
    for (int i = 0; i < vec.size(); ++i)
        ret.push_back(vec[i] / sc);

    return ret;
}

template<typename T>
std::vector<T> VectorAction::operator+(const std::vector<T>& vec1, const std::vector<T>& vec2)
{
    if (vec1.size() != vec2.size())
        throw std::out_of_range("Vector ERROR Sum Operator: vec1.size() != vec2.size()");


    std::vector<T> ret;
    for (int i = 0; i < vec1.size(); ++i)
        ret.push_back(vec1[i] + vec2[i]);

    return ret;
}

template<typename T>
std::vector<T> VectorAction::operator-(const std::vector<T>& vec1, const std::vector<T>& vec2)
{
    if (vec1.size() != vec2.size())
        throw std::out_of_range("Vector ERROR Sum Operator: vec1.size() != vec2.size()");


    std::vector<T> ret;
    for (int i = 0; i < vec1.size(); ++i)
        ret.push_back(vec1[i] - vec2[i]);

    return ret;
}

template<typename T>
double VectorAction::abs(const std::vector<T>& vec)
{
    double ret = vec * vec;
    return std::sqrt(ret);
}

template<typename T>
std::ostream& VectorAction::operator<<(std::ostream& os, const std::vector<T>& vec)
{
    std::ios::fmtflags old_flags = os.flags();

    for (std::size_t i = 0; i < vec.size(); ++i)
    {
        os << std::fixed << std::setprecision(10) << vec.at(i);
        if (i < vec.size() - 1)
            os << ',';
    }
    os.setf(old_flags);
    return os;
}

template<typename T>
std::vector<T> VectorAction::operator-(const std::vector<T>& vec)
{
    std::vector<T> ret;
    for (auto& i : vec)
        ret.push_back(-i);
    return ret;
}


#endif //ROTATION2D_VECTORACTION_H
