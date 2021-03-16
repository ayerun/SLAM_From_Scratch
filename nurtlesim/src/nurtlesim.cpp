#include <nurtlesim/nurtlesim.hpp>

namespace cl
{
    double dist(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2)
    {
        double dx = p2.x-p1.x;
        double dy = p2.y-p1.y;
        double dr = sqrt(pow(dx,2)+pow(dy,2));
        return dr;
    }

    double calculateD(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2)
    {
        double D = p1.x*p2.y-p2.x*p1.y;
        return D;
    }

    int sgn(const double x)
    {
        if(x<0)
        {
            return -1;
        }
        else
        {
            return 1;
        }  
    }

    rigid2d::Vector2D findIntersection(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2, const double r)
    {
        //initialize
        rigid2d::Vector2D intersect1;
        rigid2d::Vector2D intersect2;

        //setup
        double dx = p2.x-p1.x;
        double dy = p2.y-p1.y;
        double dr = sqrt(pow(dx,2)+pow(dy,2));
        double D = calculateD(p1,p2);
        double disc = discriminant(p1,p2,r);

        //calculate x coordinates
        intersect1.x = (D*dy+sgn(dy)*dx*sqrt(disc))/pow(dr,2);
        intersect2.x = (D*dy-sgn(dy)*dx*sqrt(disc))/pow(dr,2);

        //calculate y coordinates
        intersect1.y = (-D*dx+fabs(dy)*sqrt(disc))/pow(dr,2);
        intersect2.y = (-D*dx-fabs(dy)*sqrt(disc))/pow(dr,2);

        //find distance to points of intersection
        double d1 = dist(p1,intersect1);
        double d2 = dist(p1,intersect2);

        //return closest point
        if(d1<d2)
        {
            return intersect1;
        }
        else
        {
            return intersect2;
        }
        
    }

    double discriminant(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2, const double r)
    {
        double dr = dist(p1,p2);
        double D = calculateD(p1,p2);
        double disc = pow(r,2)*pow(dr,2)-pow(D,2);
        return disc;
    }

    rigid2d::Vector2D line(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2)
    {
        rigid2d::Vector2D params;
        params.x = (p2.y-p1.y)/(p2.x-p1.x);     //slope
        params.y = p1.y-params.x*p1.x;          //y intercept

        return params;
    }

    rigid2d::Vector2D line_intersect(const rigid2d::Vector2D line1, const rigid2d::Vector2D line2)
    {
        rigid2d::Vector2D intersect;
        intersect.x = (line2.y-line1.y)/(line1.x-line2.x);
        intersect.y = line1.x*intersect.x+line1.y;
        
        return intersect;
    }
}