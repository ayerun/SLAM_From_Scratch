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

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        is >> v.x;
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> v.x;
        }
        
        is >> v.y;
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> v.y;
        }
        
        return is;
    }

    Transform2D::Transform2D()
    {
        
    }
}
//
int main()
{
    //test <<
    rigid2d::Vector2D v;
    v.x = 4;
    v.y = 5;
    std::cout<<v<<std::endl;

    //test >>
    rigid2d::Vector2D myVec;
    std::cin>>myVec;
    std::cout<<myVec;


    return 0;
}