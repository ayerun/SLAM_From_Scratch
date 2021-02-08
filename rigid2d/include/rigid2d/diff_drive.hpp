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
        /// \param trans - Transform from world frame to body frame
        DiffDrive(double b, double r, Transform2D trans);

        /// \brief convert twist to wheel velocities
        /// \param Vb - desired twist
        /// \return Vector of wheel velocities [left_velocity, right_velocity]
        Vector2D calculateControls(Twist2D Vb);

        /// \brief convert wheel velocities to twist
        /// \param u - robot controls [left_velocity, right_velocity]
        /// \return body twist
        Twist2D calculateTwist(Vector2D u);

        /// \brief update robot configuration
        /// \param angs - Vector of updated wheel angles [left_angle, right_angle]
        void updateConfiguration(Vector2D angs);

        /// \brief accessor function for private members
        /// \return base
        double getBase();

        /// \brief accessor function for private members
        /// \return radius
        double getRadius();

        /// \brief accessor function for private members
        /// \return Twb
        Transform2D getTransform();

    private:
        double base;
        double radius;
        Transform2D Twb;
    };
}

#endif