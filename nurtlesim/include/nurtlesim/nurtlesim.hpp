/// \file
/// \brief Library for circle-line intersection.

#ifndef NURTLESIM_INCLUDE_GUARD_HPP
#define NURTLESIM_INCLUDE_GUARD_HPP

#include <cmath>
#include <rigid2d/rigid2d.hpp>

namespace cl
{
    double dist(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2);

    double calculateD(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2);

    int sgn(const double x);

    rigid2d::Vector2D findIntersection(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2, const double r);

    double discriminant(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2, const double r);
}


#endif