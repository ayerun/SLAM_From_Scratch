#include "rigid2d.hpp"
// #include <cmath>
#include <iostream>

namespace rigid2d
{
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        const double x = v.x;
        const double y = v.y;

        os << "[" << x << " " << y << "]" << std::endl;

        return os;
    }
}
int main()
{
    rigid2d::Vector2D v;
    v.x = 4;
    v.y = 5;
    std::cout<<v;
    return 0;
}