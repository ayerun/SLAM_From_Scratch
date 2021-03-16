/// \file
/// \brief Library for circle-line intersection.

#ifndef NURTLESIM_INCLUDE_GUARD_HPP
#define NURTLESIM_INCLUDE_GUARD_HPP

#include <cmath>
#include <rigid2d/rigid2d.hpp>

namespace cl
{
    /// \brief calculate the distance between points
    /// \param p1 - point 1
    /// \param p2 - point 2
    /// \returns distance
    double dist(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2);

    /// \brief calculate the determinant
    /// \param p1 - point 1
    /// \param p2 - point 2
    /// \returns determinant
    double calculateD(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2);

    /// \brief sign function
    /// \param x - value
    /// \returns sign
    int sgn(const double x);

    /// \brief finds points of intersections between a circle and a line
    /// \param p1 - point on line
    /// \param p2 - point on line
    /// \param r - circle radius
    /// \returns closest point of intersection
    rigid2d::Vector2D findIntersection(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2, const double r);

    /// \brief calculates discriminant
    /// \param p1 - point on line
    /// \param p2 - point on line
    /// \param r - circle radius
    double discriminant(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2, const double r);

    /// \brief calculate slope and y-intercept
    /// \param p1 - point on line
    /// \param p2 - point on line
    /// \returns [slope y-intercept]
    rigid2d::Vector2D line(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2);

    /// \brief calculates point of intersection between 2 lines
    /// \param line1 - slope and y-intercept of line 1
    /// \param line2 - slope and y-intercept of line 2
    /// \returns point of intersection
    rigid2d::Vector2D line_intersect(const rigid2d::Vector2D line1, const rigid2d::Vector2D line2);
}


#endif