#include <rigid2d/rigid2d.hpp>
#include <cmath>
#include <iostream>

namespace rigid2d
{
    Vector2D::Vector2D()
    {
        x = 0;
        y = 0;
    }

    Vector2D::Vector2D(double x_in, double y_in)
    {
        x = x_in;
        y = y_in;
    }

    Vector2D Vector2D::normalize() const
    {
        Vector2D norm_v;
        double len;

        len = sqrt(pow(x,2)+pow(y,2));
        norm_v.x = x/len;
        norm_v.y = y/len;

        return norm_v;
    }

    double Vector2D::dot(const Vector2D & rhs) const
    {
        double dp = x*rhs.x+y*rhs.y;
        return dp;
    }

    Vector2D & Vector2D::operator*=(const double & rhs)
    {
        x = rhs*x;
        y = rhs*y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs)
    {
        x = x-rhs.x;
        y = y-rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs)
    {
        x = x+rhs.x;
        y = y+rhs.y;
        return *this;
    }

    Vector2D operator*(double rhs, Vector2D & lhs)
    {
        Vector2D v;
        v = lhs;

        v*=rhs;
        return v;
    }

    Vector2D operator*(Vector2D & rhs, double lhs)
    {
        Vector2D v;
        v = rhs;

        v*=lhs;
        return v;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
    {
        Vector2D v;
        v = lhs;

        v+=rhs;
        return v;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
    {
        Vector2D v;
        v = lhs;

        v-=rhs;
        return v;
    }

    bool operator==(Vector2D lhs, const Vector2D & rhs)
    {
        if (lhs.x == rhs.x && lhs.y == rhs.y) return true;
        else return false;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        const double x = v.x;
        const double y = v.y;

        os << "[" << x << " " << y << "]" << std::endl;

        return os;
    }

    //created this function with Nathaniel Nyberg and Sarah Ziselman
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

    double magnitude(const Vector2D & v)
    {
        double mag;

        mag = sqrt(pow(v.x,2)+pow(v.y,2));

        return mag;
    }

    double angle(const Vector2D & v)
    {
        double ang;

        ang = atan(v.y/v.x);

        if(v.x<0 && v.y>=0)
        {
            if(v.y == 0)
            {
                ang = PI;
            }
            else
            {
                ang = -ang+(PI/2);
            }
        }

        else if(v.x<=0 && v.y<0)
        {
            if(v.x == 0)
            {
                ang = ang+2*PI;
            }
            else
            {
                ang = ang+PI;
            }
        }

        else if(v.x>0 && v.y<0)
        {
            ang = -ang+(3*PI/2);
        }

        return ang;
    }

    double angle(const Vector2D v1, const Vector2D v2)
    {
        double theta = acos( (v1.dot(v2)) / (magnitude(v1)*magnitude(v2)) );
        return theta;
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
        th = normalize_angle(radians);
        sinth = sin(radians);
        costh = cos(radians);
    }

    Transform2D::Transform2D(const Vector2D & trans, double radians)
    {
        x = trans.x;
        y = trans.y;
        th = normalize_angle(radians);
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

    double Transform2D::getX() const
    {
        return x;
    }

    double Transform2D::getY() const
    {
        return y;
    }

    double Transform2D::getTheta() const
    {
        return th;
    }

    double Transform2D::getCtheta() const
    {
        return costh;
    }

    double Transform2D::getStheta() const
    {
        return sinth;
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
        Transform2D T;
        T = lhs;

        T*=rhs;
        return T;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        double theta;
        
        Vector2D vec;

        is >> theta;
        theta = deg2rad(theta);
        while(is.fail())
        {
            is.clear();
            is.ignore(1);
            is >> theta;
            theta = deg2rad(theta);
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

    Transform2D integrateTwist(const Twist2D & twist)
    {
        Transform2D Tbb_;

        if(almost_equal(twist.w,0))
        {
            Vector2D v;
            
            v.x = twist.x_dot;
            v.y = twist.y_dot;
            Tbb_ = Transform2D(v);
        }

        else
        {
            Transform2D Tbs_;
            Vector2D bs;
            Transform2D Tss_;
            Transform2D Tsb;
            Transform2D Tbs;
            Transform2D Tb_s_;

            bs.y = -twist.x_dot/twist.w;
            bs.x = twist.y_dot/twist.w;
            Tbs = Transform2D(bs);
            Tss_ = Transform2D(twist.w);
            Tb_s_ = Transform2D(bs);
            Tbb_ = Tbs.inv()*Tss_*(Tb_s_);  //fixed error in lecture
        }

        return Tbb_;
    }

    double normalize_angle(double rad)
    {
        while(rad > PI)
        {
            rad = rad-2*PI;
        }
        while(rad < -PI)
        {
            rad = rad+2*PI;
        }
        return rad;
    }
}