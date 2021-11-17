#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath>

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (fabs(d1-d2) < epsilon)
        {
            return true;
        }
        else
        {
            return false;
        }

    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
        return (deg*PI)/180;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return (rad*180)/PI;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    static_assert(almost_equal(0, 3, 4), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(180), PI), "deg2rad failed");
    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");
    static_assert(almost_equal(rad2deg(PI), 180), "rad2deg failed");


    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x = 0.0;
        double y = 0.0;

        /// \brief Create an 0 magnitude vector
        Vector2D();

        /// \brief Create a vector with specified components
        /// \param x_in - x component
        /// \param y_in - y component
        Vector2D(double x_in, double y_in);
        
        /// \brief normalize 2D vector
        /// \return normalized vector
        Vector2D normalize() const;

        /// \brief scalar multiplication of a vector
        /// \param rhs - scalar
        /// \returns a reference to the newly scaled vector
        Vector2D & operator*=(const double & rhs);

        /// \brief vector subtraction
        /// \param rhs - vector to subtract
        /// \returns a reference to the difference between vectors
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief vector addition
        /// \param rhs - vector to add
        /// \returns a reference to the sum of vectors
        Vector2D & operator+=(const Vector2D & rhs);
    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent, ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief scalar multiplication of vector
    /// \param lhs - scalar
    /// \param rhs - vector to scale
    /// \return the scaled vector
    Vector2D operator*(double lhs, Vector2D & rhs);

    /// \brief scalar multiplication of vector
    /// \param lhs - scalar
    /// \param rhs - vector to scale
    /// \return the scaled vector
    Vector2D operator*(Vector2D & rhs, double lhs);

    /// \brief vector addition
    /// \param lhs - vector 1
    /// \param rhs - vector 2
    /// \return sum of vectors
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief vector addition
    /// \param lhs - left hand side vector
    /// \param rhs - right hand side vector
    /// \return difference between vectors
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief vector equality
    /// \param lhs - left hand side vector
    /// \param rhs - right hand side vector
    /// \return true if vectors have same components
    bool operator==(Vector2D lhs, const Vector2D & rhs);

    /// \brief calculate magnitude of 2D vector
    /// \return magnitude of vector
    double magnitude(const Vector2D & v);

    /// \brief calculate angle of 2D vector
    /// \return angle of vector in radians
    double angle(const Vector2D & v);

    /// \brief A 2-Dimensional Twist
    struct Twist2D
    {
        double w = 0;       //angular velocity
        double x_dot = 0;   //linear x velocity
        double y_dot = 0;   //linear y velocity
    };

    /// \brief output a 2 dimensional twist as [xcomponent ycomponent]
    /// os - stream to output to
    /// t - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & t);

    /// \brief input a 2 dimensional twist
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent, ycomponent]
    /// is - stream from which to read
    /// t [out] - output twist
    std::istream & operator>>(std::istream & is, Twist2D & v);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief convert a twist to a different reference frame
        /// \param t - the vector to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D t) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

        /// \brief accessor function for private members: used for testing
        /// \return x
        double getX() const;

        /// \brief accessor function for private members: used for testing
        /// \return y
        double getY() const;

        /// \brief accessor function for private members: used for testing
        /// \return th
        double getTheta() const;

        /// \brief accessor function for private members: used for testing
        /// \return costh
        double getCtheta() const;

        /// \brief accessor function for private members: used for testing
        /// \return sinth
        double getStheta() const;
    
    private:
        double x;
        double y;
        double th;
        double sinth;
        double costh;
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief computes transformation corresponding to a rigid body following a constant twist in its original body frame for one unit time
    /// \param twist - twist to integrate
    /// \return transformation to new frame after twist
    Transform2D integrateTwist(const Twist2D & twist);

    /// \brief converts a angle to [-PI PI]
    /// \param twist - twist to integrate
    /// \return transformation to new frame after twist
    double normalize_angle(double rad);

}

#endif
