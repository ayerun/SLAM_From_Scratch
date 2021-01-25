#include "rigid2d.hpp"
#include <cmath>
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
        x = 0;
        y = 0;
        th = 0;
        sinth = 0;
        costh = 1;
    }

    Transform2D::Transform2D(const Vector2D & trans)
    {
        x = trans.x;
        y = trans.y;
        th = 0;
        sinth = 0;
        costh = 1;
    }

    Transform2D::Transform2D(double radians)
    {
        x = 0;
        y = 0;
        th = radians;
        sinth = sin(radians);
        costh = cos(radians);
    }

    Transform2D::Transform2D(const Vector2D & trans, double radians)
    {
        x = trans.x;
        y = trans.y;
        th = radians;
        sinth = sin(radians);
        costh = cos(radians);
    }

    Vector2D Transform2D::operator()(Vector2D v) const  
    {
        Vector2D tVec;

        tVec.x = v.x*costh-v.y*sinth+x;
        tVec.y = v.x*sinth+v.y*costh+y;

        return tVec;
    }

    Transform2D Transform2D::inv() const
    {
        Transform2D inverse;
        Vector2D trans;
        double rad;
        rad = 2*PI-th;

        trans.x = -y*sinth-x*costh;
        trans.y = -y*costh+x*sinth;

        inverse = Transform2D(trans,rad);

        return inverse;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        th = th + rhs.th;
        x = rhs.x*costh-rhs.y*sinth+x;
        y = rhs.x*sinth+rhs.y*costh+y;
        costh = cos(th);
        sinth = sin(th);

        return *this;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        const double x = tf.x;
        const double y = tf.y;
        const double th = tf.th;
        const double sinth = tf.sinth;
        const double costh = tf.costh;

        os << "delta x = " << x << std::endl;
        os << "delta y = " << y << std::endl;
        os << "delta theta = " << th << std::endl;
        os << "sin(delta theta) = " << sinth << std::endl;
        os << "cos(delta theta) = " << costh << std::endl;

        return os;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        lhs*=rhs;
        return lhs;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        double theta;
        
        Vector2D vec;

        is >> theta;
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> theta;
        }
        
        is >> vec.x;
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> vec.x;
        }

        is >> vec.y;
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> vec.y;
        }

        tf = Transform2D(vec, theta);
        
        return is;
    }
}

// int main()
// {
//     using namespace rigid2d;
//     using namespace std;

    //test <<
    // Vector2D v;
    // v.x = 4;
    // v.y = 5;
    // cout<<v<<endl;

    //test >>
    // Vector2D myVec;
    // cin>>myVec;
    // cout<<myVec;

    //test identity
    // Transform2D myTrans = Transform2D();
    // cout<<myTrans;

    //test translation
    // Vector2D v2;
    // v2.x = 4;
    // v2.y = 5;
    // Transform2D myTrans2 = Transform2D(v2);
    // cout<<myTrans;

    //test rotation
    // Transform2D myTrans3 = Transform2D(3.14159);
    // cout<<myTrans3;

    //test translation and rotation
    // Vector2D v3;
    // v3.x = 0;
    // v3.y = 0;
    // Transform2D myTrans4 = Transform2D(v3,2);
    // cout<<myTrans4;

    //test ()
    //Transform2D(Vector2D) = Vector2D
    // Vector2D v4;
    // v4.x = 1;
    // v4.y = 0;
    // auto output = myTrans4(v4);
    // cout<<output;

    // test inv()
    // Transform2D inverse;
    // cout<<myTrans4<<endl;
    // inverse = myTrans4.inv();
    // cout<<inverse<<endl;
    // inverse = inverse.inv();
    // cout<<inverse<<endl;

    // test *=
    // Vector2D v5;
    // v5.x = 1;
    // v5.y = 1;
    // Transform2D Tab = Transform2D(v5,0);
    // Transform2D Tbc = Transform2D(v5,PI/2);
    // Transform2D Tac;
    // Tac = Tab*Tbc;
    // Tab*=Tbc;
    // cout << Tab;
    // cout << Tac;

    // test isteam
    // Transform2D myTrans5;
    // cin>>myTrans5;
    // cout<<myTrans5;



//     return 0;
// }