#include "rigid2d.hpp"
#include <cmath>
#include <iostream>

constexpr double PI=3.14159265358979323846;

constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
{
    //true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here

    // if (fabs(d1-d2) < epsilon)
    // {
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }

    // constexpr bool output=true;
    return true;
}

// almost_equal(0,0);