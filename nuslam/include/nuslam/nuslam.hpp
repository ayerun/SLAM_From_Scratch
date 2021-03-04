/// \file
/// \brief Library for SLAM using an extended kalman filter

#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP

#include <armadillo>
#include <cmath>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

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
        arma::mat calculateA(rigid2d::Twist2D ut);

        /// \brief calculates Q bar matrix (process noise for robot motion model, expanding to fill the whole state)
        arma::mat calculateQbar();

        /// \brief
        arma::mat predictUncertainty(arma::mat A, arma::mat Q_bar);

        /// \brief
        int getN();

        /// \brief
        arma::mat getSigma();

        /// \brief
        arma::mat getState();
        


    private:
        int n;                      //maximum number of landmarks
        arma::mat sigma;            //covariance matrix
        arma::mat state;            //system state (robot location & landmarks)
        arma::mat Q;                //process noise covariance matrix
        arma::mat R;                //sensor noise covariance matrix
    };
}


#endif