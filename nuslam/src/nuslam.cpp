#include <nuslam/nuslam.hpp>

namespace nuslam
{
    ekf::ekf()
    {
        n = 5;

        //sigma
        arma::mat sig0q = arma::mat(3,3,arma::fill::zeros); //not sure if this should be all zeros
        arma::mat sig2n3 = arma::mat(2*n,3,arma::fill::zeros);
        arma::mat sig32n = arma::mat(3,2*n,arma::fill::zeros);
        arma::mat sig0m = 999999*arma::mat(2*n,2*n,arma::fill::eye);
        auto joined_ab  = arma::join_cols(sig0q,sig2n3);
        auto joined_cd  = arma::join_cols(sig32n,sig0m);
        sigma = arma::join_rows(joined_ab,joined_cd);

        //state
        state = arma::mat(2*n+3,1,arma::fill::zeros);

        //covariances
        Q = { {0.1, 0.08, 0.12},
              {0.08, 0.1, 0.14},
              {0.12, 0.14, 0} };
        R = { {0.1, 0.08, 0.12},
              {0.08, 0.1, 0.14},
              {0.12, 0.14, 0} };
    }

    ekf::ekf(int max_n)
    {
        n = max_n;

        //sigma
        arma::mat sig0q = arma::mat(3,3,arma::fill::zeros); //not sure if this should be all zeros
        arma::mat sig2n3 = arma::mat(2*n,3,arma::fill::zeros);
        arma::mat sig32n = arma::mat(3,2*n,arma::fill::zeros);
        arma::mat sig0m = 999999*arma::mat(2*n,2*n,arma::fill::eye);
        auto joined_ab  = arma::join_cols(sig0q,sig2n3);
        auto joined_cd  = arma::join_cols(sig32n,sig0m);
        sigma = arma::join_rows(joined_ab,joined_cd);

        //state
        state = arma::mat(2*n+3,1,arma::fill::zeros);

        //covariances
        Q = { {0.1, 0.08, 0.12},
              {0.08, 0.1, 0.14},
              {0.12, 0.14, 0} };
        R = { {0.1, 0.08, 0.12},
              {0.08, 0.1, 0.14},
              {0.12, 0.14, 0} };
    }

    ekf::ekf(int max_n, arma::mat Q_mat, arma::mat R_mat)
    {
        n = max_n;

        //sigma
        arma::mat sig0q = arma::mat(3,3,arma::fill::zeros); //not sure if this should be all zeros
        arma::mat sig2n3 = arma::mat(2*n,3,arma::fill::zeros);
        arma::mat sig32n = arma::mat(3,2*n,arma::fill::zeros);
        arma::mat sig0m = 999999*arma::mat(2*n,2*n,arma::fill::eye);
        auto joined_ab  = arma::join_cols(sig0q,sig2n3);
        auto joined_cd  = arma::join_cols(sig32n,sig0m);
        sigma = arma::join_rows(joined_ab,joined_cd);

        //state
        state = arma::mat(2*n+3,1,arma::fill::zeros);

        //covariances
        Q = Q_mat;
        R = R_mat;
    }

    arma::mat ekf::calculateA(rigid2d::Twist2D ut)
    {
        arma::mat A;
        if(ut.w == 0)
        {
            arma::mat I = arma::mat(2*n+3,2*n+3,arma::fill::eye);
            arma::mat a = { {0,0,0},
                            {-ut.x_dot*sin(state(0,0)),0,0},
                            {ut.x_dot*cos(state(0,0)),0,0} };
            arma::mat b = arma::mat(3,2*n,arma::fill::zeros);
            arma::mat c = arma::mat(2*n,3,arma::fill::zeros);
            arma::mat d = arma::mat(2*n,2*n,arma::fill::zeros);
            auto joined_ab  = arma::join_rows(a,b);
            auto joined_cd  = arma::join_rows(c,d);
            A = arma::join_cols(joined_ab,joined_cd)+I;
        }
        else
        {
            arma::mat I = arma::mat(2*n+3,2*n+3,arma::fill::eye);
            arma::mat a = { {0,0,0},
                            {-(ut.x_dot/ut.w)*cos(state(0,0))+(ut.x_dot/ut.w)*cos(state(0,0)+ut.w),0,0},
                            {-(ut.x_dot/ut.w)*sin(state(0,0))+(ut.x_dot/ut.w)*sin(state(0,0)+ut.w),0,0} };
            arma::mat b = arma::mat(3,2*n,arma::fill::zeros);
            arma::mat c = arma::mat(2*n,3,arma::fill::zeros);
            arma::mat d = arma::mat(2*n,2*n,arma::fill::zeros);
            auto joined_ab  = arma::join_rows(a,b);
            auto joined_cd  = arma::join_rows(c,d);
            A = arma::join_cols(joined_ab,joined_cd)+I;
        }
        return A;
    }

    arma::mat ekf::calculateQbar()
    {
        arma::mat b = arma::mat(3,2*n,arma::fill::zeros);
        arma::mat c = arma::mat(2*n,3,arma::fill::zeros);
        arma::mat d = arma::mat(2*n,2*n,arma::fill::zeros);
        auto joined_ab  = arma::join_rows(Q,b);
        auto joined_cd  = arma::join_rows(c,d);
        arma::mat Q_bar = arma::join_cols(joined_ab,joined_cd);
        return Q_bar;
    }

    arma::mat ekf::predictUncertainty(rigid2d::Twist2D ut)
    {
        arma::mat A = this->calculateA(ut);
        arma::mat Qbar = this->calculateQbar();
        arma::mat sigma_estimate = A*sigma*A.t()+Qbar;

        return sigma_estimate;
    }

    int ekf::getN()
    {
        return n;
    }

    arma::mat ekf::getSigma()
    {
        return sigma;
    }

    arma::mat ekf::getState()
    {
        return state;
    }

    arma::mat ekf::getQ()
    {
        return Q;
    }

    arma::mat ekf::getR()
    {
        return R;
    }
}