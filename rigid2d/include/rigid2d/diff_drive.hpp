#ifdef RIGID2D_INCLUDE_GUARD_HPP

/// \file
/// \brief Library for differential drive kinematics

namespace rigid2d
{
    class DiffDrive
    {
    public:
        /// \brief default DiffDrive constructor
        DiffDrive();

        /// \brief create a DiffDrive object at origin
        /// \param b - wheel base 
        /// \param r - wheel radius
        DiffDrive(double b, double r);

        /// \brief create a DiffDrive object at specified location
        /// \param b - wheel base 
        /// \param r - wheel radius
        /// \param x_coor - x coordinate
        /// \param y_coor - y coordinate
        /// \param theta - rotation
        DiffDrive(double b, double r, double x_coor, double y_coor, double theta);

        /// \brief convert twist to wheel velocities
        /// \param Vb - desired twist
        /// \return Vector of wheel velocities [left_w_velocity, right_wheel_velocity]
        Vector2D calculateVelocity(Twist2D Vb);
        //write body frame twist in each wheel frame

        /// \brief update robot configuration
        /// \param angs - list of updated wheel angles [left_angle, right_angle]
        void updateConfiguration(double angs);

        /// \brief accessor function for private members
        /// \return base
        double getBase();

        /// \brief accessor function for private members
        /// \return radius
        double getRadius();

        /// \brief accessor function for private members
        /// \return x
        double getX();

        /// \brief accessor function for private members
        /// \return y
        double getY();

        /// \brief accessor function for private members
        /// \return theta
        double getTheta();

    private:
        double base;
        double radius;
        double x;
        double y;
        double th;
    };
}

#endif