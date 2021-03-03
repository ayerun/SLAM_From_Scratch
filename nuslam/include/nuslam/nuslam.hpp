/// \file
/// \brief Library for SLAM using an extended kalman filter

//eps_t is column vector of configuration vars and landmark locations
//u_t is column vector of delta theta and delta x and delta y (0)
//

#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP

#include <armadillo>
#include <cmath>

namespace nuslam
{
    /// \brief
    class ekf
    {
    public:
        /// \brief
        ekf();

        

        /// \brief
        int getN();
        
        /// \brief
        arma::mat getA();


    private:
        int n;          //maximum number of landmarks
        arma::mat A;    //derivative
        arma::mat sigma;

    };
}


#endif