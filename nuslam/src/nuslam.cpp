// #include <nuslam/nuslam.hpp>
#include "../include/nuslam/nuslam.hpp"

namespace nuslam
{
    ekf::ekf()
    {
        n = 1;
        A = arma::mat(5,5,arma::fill::ones);
    }

    int ekf::getN()
    {
        return n;
    }

    arma::mat ekf::getA()
    {
        return A;
    }
}