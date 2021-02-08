#include "../include/rigid2d/rigid2d.hpp"
#include "../include/rigid2d/diff_drive.hpp"
#include <iostream>

namespace rigid2d
{
    DiffDrive::DiffDrive()
    {
        base = 0;
        radius = 0;
        Twb = Transform2D();
    }

    DiffDrive::DiffDrive(double b, double r)
    {
        base = b;
        radius = r;
        Twb = Transform2D();
    }

    DiffDrive::DiffDrive(double b, double r, Transform2D trans)
    {
        base = b;
        radius = r;
        Twb = trans;
    }

    Vector2D DiffDrive::calculateControls(Twist2D Vb)
    {
        Vector2D controls;
        double D = base/2;

        controls.x = (-D*Vb.w+Vb.x_dot)/radius;
        controls.y = (D*Vb.w+Vb.x_dot)/radius;

        return controls;
    }

    Twist2D DiffDrive::calculateTwist(Vector2D u)
    {
        Twist2D T;
        double D = base/2;

        T.w = radius*(u.y-u.x)/(2*D);
        T.x_dot = radius*(u.y+u.x)/(2);
        T.y_dot = 0;

        return T;
    }

    void DiffDrive::updateConfiguration(Vector2D angs)
    {
        Twist2D Vb;
        Twist2D dq;
        Transform2D Tbb_;
        Vector2D trans;

        //calculate body twist
        Vb = calculateTwist(angs);

        //integrate twist to find Tbb'
        Tbb_ = integrateTwist(Vb);

        //store Tbb' into twist dq
        dq.w = Tbb_.getTheta()+Twb.getTheta();
        dq.x_dot = Tbb_.getX();
        dq.y_dot = Tbb_.getY();

        //use adjoint to convert twist from body frame to world frame
        trans.x = (dq.x_dot*Twb.getCtheta()-dq.y_dot*Twb.getStheta())+Twb.getX();
        trans.y = (dq.x_dot*Twb.getStheta()+dq.y_dot*Twb.getCtheta())+Twb.getY();

        //update private member
        Twb = Transform2D(trans,dq.w);
    }

    double DiffDrive::getBase()
    {
        return base;
    }

    double DiffDrive::getRadius()
    {
        return radius;
    }

    Transform2D DiffDrive::getTransform()
    {
        return Twb;
    }
}