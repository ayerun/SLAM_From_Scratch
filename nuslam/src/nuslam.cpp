#include <nuslam/nuslam.hpp>

namespace nuslam
{
    ekf::ekf()
    {
        n = 5;

        //sigma
        arma::mat sig0q = { {0.1,0.1,0.1},
                            {0.1,0.1,0.1},
                            {0.1,0.1,0.1}};
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
        R = { {0.1, 0.08},
              {0.08, 0.1} };
    }

    ekf::ekf(int max_n)
    {
        n = max_n;

        //sigma
        arma::mat sig0q = { {0.0,0.0,0.0},
                            {0.0,0.0,0.0},
                            {0.0,0.0,0.0}};
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
        R = { {0.1, 0.08},
              {0.08, 0.1} };
    }

    ekf::ekf(int max_n, arma::mat Q_mat, arma::mat R_mat)
    {
        n = max_n;

        //sigma
        arma::mat sig0q = { {0.0,0.0,0.0},
                            {0.0,0.0,0.0},
                            {0.0,0.0,0.0}};
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

    void ekf::predict(rigid2d::Twist2D ut,rigid2d::Transform2D odom)
    {
        //predict uncertainty
        arma::mat A = this->calculateA(ut);
        arma::mat Qbar = this->calculateQbar();
        arma::mat sigma_estimate = A*sigma*A.t()+Qbar;
        sigma = sigma_estimate;

        //update state estimate
        state_odom = state;
        state_odom(0,0) = odom.getTheta();
        state_odom(0,1) = odom.getX();
        state_odom(0,2) = odom.getY();
    }

    arma::mat ekf::calculateH(rigid2d::Vector2D m, int j)
    {
        rigid2d::Vector2D point = rigid2d::Vector2D(state(0,1),state(0,2));
        rigid2d::Vector2D d = m-point;
        rigid2d::Vector2D d_norm = d.normalize();
        double d_mag = pow(d.x,2)+pow(d.y,2);

        arma::mat A = { {0, -d_norm.x, -d_norm.y},
                        {-1, d.y/d_mag, -d.x/d_mag} };
        arma::mat B = arma::mat(2,2*(j-1),arma::fill::zeros);
        arma::mat C = { {d_norm.x, d_norm.y},
                        {-d.y/d_mag, d.x/d_mag} };
        arma::mat D = arma::mat(2,2*n-2*j,arma::fill::zeros);

        arma::mat H = arma::join_rows(A,B,C,D);

        return H;
    }

    arma::mat ekf::calculatez(rigid2d::Vector2D m, rigid2d::Vector2D point)
    {
        arma::mat z = arma::mat(2,1,arma::fill::zeros);
        double rj = sqrt(pow(m.x-state(1,0),2)-pow(m.y-state(2,0),2));
        double phij = atan2(m.y-point.x, m.x-point.y)-state(0,0);
        z(0,0) = rj;
        z(1,0) = phij;
        return z;
    }

    arma::mat ekf::calculatezhat(rigid2d::Vector2D m, rigid2d::Vector2D point)
    {
        arma::mat z = arma::mat(2,1,arma::fill::zeros);
        double rj = sqrt(pow(m.x-state_odom(1,0),2)-pow(m.y-state_odom(2,0),2));
        double phij = atan2(m.y-point.x, m.x-point.y)-state_odom(0,0);
        z(0,0) = rj;
        z(1,0) = phij;
        return z;
    }

    // void ekf::setup_update()

    void ekf::update(rigid2d::Vector2D m, rigid2d::Vector2D point, int j)
    {
        //setup
        arma::mat z = calculatez(m,point);
        arma::mat zhat = calculatezhat(m,point);
        arma::mat H = calculateH(m,j);

        //compute kalman gain
        arma::mat K = sigma*H.t()*(H*sigma*H.t()+R).i();

        //compute poseterior state update
        arma::mat diff = z-zhat;
        diff(1,0) = rigid2d::normalize_angle(diff(1,0)); //angle wraparound
        state = state_odom + K*diff;

        //compute posterior covariance
        arma::mat I = arma::mat(2*n+3,2*n+3,arma::fill::eye);
        sigma = (I-K*H)*sigma;
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

    arma::mat ekf::getState_odom()
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