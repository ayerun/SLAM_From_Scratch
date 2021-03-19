/// \file
/// \brief Library for SLAM using an extended kalman filter

#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP
#include <armadillo>
#include <rigid2d/rigid2d.hpp>

namespace nuslam
{
    /// \brief
    class ekf
    {
    public:
        /// \brief defual constructor, max landmarks set to 5
        ekf();

        /// \brief constructor given max landmarks
        /// \param max_n - max number of landmarks
        explicit ekf(int max_n);

        /// \brief constructor given max landmarks and covariance matricies
        /// \param max_n - max number of landmarks
        /// \param Q_mat - covariance matrix for process noise
        /// \param R_mat - covariance matrix for sensor noise
        ekf(int max_n, arma::mat Q_mat, arma::mat R_mat);

        /// \brief calculates A matrix (derivative of model without noise with respect to state)
        /// \param ut - twist
        /// \returns A matrix
        arma::mat calculateA(rigid2d::Twist2D ut);

        /// \brief calculates Q bar matrix (process noise for robot motion model, expanding to fill the whole state)
        /// \returns Q bar matrix
        arma::mat calculateQbar();

        /// \brief propogates uncertainty using the linearized state transition model to update sigma
        ///        update state estimate using robot odometry
        /// \param ut - twist
        /// \param odom - robot odometry
        void predict(rigid2d::Twist2D ut,rigid2d::Transform2D odom);

        /// \brief calculates H matrix (derivative of relationship between system and measurement with respect to system)
        /// \param j - landmark id
        arma::mat calculateH(int j);

        /// \brief calculate range and bearing to landmark using odom state
        arma::mat calculatezhat(int j);

        /// \param z - measurement
        /// \param j = landmark id
        void update(arma::mat z, int j);

        void initialize_landmark(int j, rigid2d::Vector2D location);

        /// \brief accessor function
        /// \returns n
        int getN();

        /// \brief accessor function
        /// \returns sigma
        arma::mat getSigma();

        /// \brief accessor function
        /// \returns state
        arma::mat getState();

        /// \brief accessor function
        /// \returns Q
        arma::mat getQ();

        /// \brief accessor function
        /// \returns R
        arma::mat getR();
        


    private:
        int n;                      //maximum number of landmarks
        arma::mat sigma;            //covariance matrix
        arma::mat state;            //system state estimate based on filter (robot location & landmarks)
        arma::mat state_odom;       //system state based on odometry (robot location & landmarks)
        arma::mat Q;                //process noise covariance matrix
        arma::mat R;                //sensor noise covariance matrix
    };

    arma::mat convert_polar(rigid2d::Vector2D point);
    rigid2d::Vector2D rb2xy(const double r, const double b);
    double dist(const rigid2d::Vector2D p1, const rigid2d::Vector2D p2);
}


#endif