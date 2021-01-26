#include "rigid2d.hpp"
#include <cmath>
#include <iostream>

namespace rigid2d
{
    Vector2D Vector2D::normalize() const
    {
        Vector2D norm_v;
        double len;

        len = sqrt(pow(x,2)+pow(y,2));
        norm_v.x = x/len;
        norm_v.y = y/len;

        return norm_v;
    }

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

    Twist2D Transform2D::operator()(Twist2D t) const
    {
        Twist2D t2;

        t2.w = t.w;
        t2.x_dot = y*t.w+costh*t.x_dot-sinth*t.y_dot;
        t2.y_dot = -x*t.w+sinth*t.x_dot+costh*t.y_dot;

        return t2;
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

    std::ostream & operator<<(std::ostream & os, const Twist2D & t)
    {
        const double w = t.w;
        const double x_dot = t.x_dot;
        const double y_dot = t.y_dot;

        os << "[" << w << " " << x_dot << " " << y_dot << "]" << std::endl;

        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & t)
    {
        is >> t.w;
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> t.w;
        }
        
        is >> t.x_dot;
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> t.x_dot;
        }

        is >> t.y_dot;
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> t.y_dot;
        }
        
        return is;
    }
}