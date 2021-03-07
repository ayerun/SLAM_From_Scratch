#include <armadillo>
#include <catch_ros/catch.hpp>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <nuslam/nuslam.hpp>

//rosrun nuslam nuslam_test

TEST_CASE("landmark constructor initialization","[landmark_constructor]"){ // Arun, Kumar
 
    int n = 2;
    nuslam::ekf filter = nuslam::ekf(n);
        
    REQUIRE(rigid2d::almost_equal(filter.getN(),2));
    REQUIRE(rigid2d::almost_equal(filter.getSigma()(0,0),0));
    REQUIRE(rigid2d::almost_equal(filter.getSigma()(6,6),999999));
    for(int i=0; i<2*n+3; i++)
    {
        REQUIRE(rigid2d::almost_equal(filter.getState()(i,0),0));
    }
}

TEST_CASE("calculate A no rotation","[calculate_A]"){ // Arun, Kumar
 
    int n = 2;
    nuslam::ekf filter = nuslam::ekf(n);
    rigid2d::Twist2D ut;
    ut.w = 0;
    ut.x_dot = 2;

    arma::mat A = filter.calculateA(ut);
    for(int i=0; i<2*n+3; i++)
    {
        for(int j=0; j<2*n+3; j++)
        {
            if(i==2 && j==0)
            {
                REQUIRE(rigid2d::almost_equal(A(i,j),2));
            }
            else if(i==j)
            {
                REQUIRE(rigid2d::almost_equal(A(i,j),1));
            }
            else
            {
                REQUIRE(rigid2d::almost_equal(A(i,j),0));
            }
            
        }
    }
}

TEST_CASE("calculate A with rotation","[calculate_A]"){ // Arun, Kumar
 
    int n = 2;
    nuslam::ekf filter = nuslam::ekf(n);
    rigid2d::Twist2D ut;
    ut.w = rigid2d::PI;
    ut.x_dot = 2;

    arma::mat A = filter.calculateA(ut);
    for(int i=0; i<2*n+3; i++)
    {
        for(int j=0; j<2*n+3; j++)
        {
            if(i==1 && j==0)
            {
                REQUIRE(rigid2d::almost_equal(A(i,j),-4/rigid2d::PI));
            }
            else if(i==j)
            {
                REQUIRE(rigid2d::almost_equal(A(i,j),1));
            }
            else
            {
                REQUIRE(rigid2d::almost_equal(A(i,j),0));
            }
            
        }
    }
}

TEST_CASE("calculate Qbar","[calculateQbar]"){ // Arun, Kumar
 
    int n = 2;
    arma::mat Q = arma::mat(3,3,arma::fill::ones);
    arma::mat R = arma::mat(3,3,arma::fill::ones);
    nuslam::ekf filter = nuslam::ekf(n,Q,R);

    arma::mat Qbar = filter.calculateQbar();
    for(int i=0; i<2*n+3; i++)
    {
        for(int j=0; j<2*n+3; j++)
        {
            if(i<3 && j<3)
            {
                REQUIRE(rigid2d::almost_equal(Qbar(i,j),1));
            }
            else
            {
                REQUIRE(rigid2d::almost_equal(Qbar(i,j),0));
            }
            
        }
    }
}

// TEST_CASE("predict Uncertainty test","[predictUncertainty]"){ // Arun, Kumar
//     //not a great test
//     int n = 2;
//     nuslam::ekf filter = nuslam::ekf(n);
//     rigid2d::Twist2D ut;
//     ut.w = rigid2d::PI;
//     ut.x_dot = 2;

//     arma::mat sigma_estimate = filter.predictUncertainty(ut);
//     arma::mat test_mat = arma::mat(2*n+3,2*n+3,arma::fill::zeros);

//     REQUIRE(arma::size(sigma_estimate)==arma::size(test_mat));
// }

// TEST_CASE("calculate H","[calculateH]"){ // Arun, Kumar
//     //not a great test
//     int j = 2;
//     int n = 2;
//     rigid2d::Vector2D m = rigid2d::Vector2D(0,0);
//     rigid2d::Vector2D point = rigid2d::Vector2D(1,0);
//     nuslam::ekf filter = nuslam::ekf(n);

//     arma::mat H = filter.calculateH(m,point,j);
//     arma::mat test_mat = arma::mat(2,2*(j-1)+(2*n-2*j)+5,arma::fill::zeros);

//     std::cout << H << std::endl;
//     REQUIRE(arma::size(H)==arma::size(test_mat));
//     REQUIRE(rigid2d::almost_equal(H(0,0),0));
//     REQUIRE(rigid2d::almost_equal(H(0,1),1));
//     REQUIRE(rigid2d::almost_equal(H(0,2),0));
//     REQUIRE(rigid2d::almost_equal(H(1,0),-1));
//     REQUIRE(rigid2d::almost_equal(H(1,1),0));
//     REQUIRE(rigid2d::almost_equal(H(1,2),1));
// }