#include <nuslam/nuslam.hpp>

namespace nuslam
{
    ekf::ekf()
    {
        n = 5;

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
        Q = { {0.1, 0.0, 0.0},
              {0.0, 0.1, 0.0},
              {0.0, 0.0, 0.1} };
        R = { {0.00001, 0},
              {0, 0.00001} };
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
        Q = { {0.1, 0.0, 0.0},
              {0.0, 0.1, 0.0},
              {0.0, 0.0, 0.1} };
        R = { {0.00001, 0},
              {0, 0.00001} };
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
        state(0,0) = odom.getTheta();
        state(1,0) = odom.getX();
        state(2,0) = odom.getY();
    }

    arma::mat ekf::calculateH(int j)
    {
        //get landmark
        rigid2d::Vector2D m = rigid2d::Vector2D(state(2*j+1,0),state(2*j+2,0));

        //get slam location
        rigid2d::Vector2D point = rigid2d::Vector2D(state(1,0),state(2,0));

        rigid2d::Vector2D d = m-point;
        rigid2d::Vector2D d_norm = d.normalize();
        double d_mag = pow(d.x,2)+pow(d.y,2);

        arma::mat A;
        arma::mat B;
        arma::mat C;
        arma::mat D;
        if (rigid2d::almost_equal(d_mag,0))
        {
            A = { {0, 0, 0},
                  {-1, 0, 0} };
            B = arma::mat(2,2*(j-1),arma::fill::zeros);
            C = { {0, 0},
                  {0, 0} };
            D = arma::mat(2,2*n-2*j,arma::fill::zeros);
        }
        else
        {
            A = { {0, -d_norm.x, -d_norm.y},
                  {-1, d.y/d_mag, -d.x/d_mag} };
            B = arma::mat(2,2*(j-1),arma::fill::zeros);
            C = { {d_norm.x, d_norm.y},
                  {-d.y/d_mag, d.x/d_mag} };
            D = arma::mat(2,2*n-2*j,arma::fill::zeros);
        }
        
        
        arma::mat H = arma::join_rows(A,B,C,D);

        return H;
    }

    arma::mat ekf::calculatezhat(int j)
    {
        //get landmark
        rigid2d::Vector2D m = rigid2d::Vector2D(state(2*j+1,0),state(2*j+2,0));

        //calculate range and bearing
        arma::mat z = arma::mat(2,1,arma::fill::zeros);
        double rj = sqrt(pow(m.x-state(1,0),2)+pow(m.y-state(2,0),2));
        double phij = atan2(m.y-state(2,0), m.x-state(1,0))-state(0,0);
        z(0,0) = rj;
        z(1,0) = phij;

        return z;
    }

    void ekf::update(arma::mat z, int j)
    {
        //setup
        arma::mat zhat = calculatezhat(j);
        arma::mat H = calculateH(j);

        //compute kalman gain
        arma::mat K = sigma*H.t()*(H*sigma*H.t()+R).i();

        //compute poseterior state update
        arma::mat diff = z-zhat;
        diff(1,0) = rigid2d::normalize_angle(diff(1,0)); //angle wraparound
        state = state + K*diff;

        //compute posterior covariance
        arma::mat I = arma::mat(2*n+3,2*n+3,arma::fill::eye);
        sigma = (I-K*H)*sigma;
    }

    void ekf::initialize_landmark(int j, rigid2d::Vector2D location)
    {
        state(2*j+1,0) = location.x;
        state(2*j+2,0) = location.y;
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

    arma::mat convert_polar(rigid2d::Vector2D point)
    {
        double r = sqrt(pow(point.x,2)+pow(point.y,2));
        double phi = atan2(point.y,point.x);
        arma::mat z = arma::mat(2,1,arma::fill::zeros);
        z(0,0) = r;
        z(1,0) = phi;
        return z;
    }
}