#include "../include/rigid2d/rigid2d.hpp"
#include "../include/rigid2d/diff_drive.hpp"
#include <iostream>

namespace rigid2d
{
    DiffDrive::DiffDrive()
    {
        base = 0;
        radius = 0;
        x = 0;
        y = 0;
        th = 0;
    }

    DiffDrive::DiffDrive(double b, double r)
    {
        base = b;
        radius = r;
        x = 0;
        y = 0;
        th = 0;
    }

    DiffDrive::DiffDrive(double b, double r, double x_coor, double y_coor, double theta)
    {
        base = b;
        radius = r;
        x = x_coor;
        y = y_coor;
        th = theta;
    }

    Vector2D DiffDrive::calculateVelocity(Twist2D Vb)
    {
        Vector2D controls;
        double D = base/2;

        controls.x = (-D*Vb.w+Vb.x_dot)/radius;
        controls.y = (D*Vb.w+Vb.x_dot)/radius;

        return controls;
    }

    double DiffDrive::getBase()
    {
        return base;
    }

    double DiffDrive::getRadius()
    {
        return radius;
    }

    double DiffDrive::getX()
    {
        return x;
    }

    double DiffDrive::getY()
    {
        return y;
    }

    double DiffDrive::getTheta()
    {
        return th;
    }
}