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