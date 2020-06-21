#include <iostream>
#include "Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() 
{

    // Create a UKF instance
    UKF ukf;

    /*
        Programming assignment calls
    */

    /* Call for Generating sigma points */
    //   MatrixXd Xsig = MatrixXd(5, 11);
    //   ukf.GenerateSigmaPoints(&Xsig);

    /* Call for Augmented process noise model */
    // MatrixXd Xsig_aug = MatrixXd(7, 15);
    // ukf.AugmentedSigmaPoints(&Xsig_aug);

    /* Call for Sigma point prediction */
    // MatrixXd Xsig_pred = MatrixXd(15, 5);
    // ukf.SigmaPointPrediction(&Xsig_pred);
    
    /* Call to predict mean and covariance from predicted sigma points */
    VectorXd x_pred = VectorXd(5);
    MatrixXd P_pred = MatrixXd(5, 5);
    ukf.PredictMeanAndCovariance(&x_pred, &P_pred);

    // print result
    //   std::cout << "Xsig = " << std::endl << Xsig << std::endl;

    return 0;
}
