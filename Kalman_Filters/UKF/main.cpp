#include <iostream>
#include "Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  /**
   * Programming assignment calls
   */
//   MatrixXd Xsig = MatrixXd(5, 11);
//   ukf.GenerateSigmaPoints(&Xsig);
  
  /* Call for Augmented process noise model */
  MatrixXd Xsig_aug = MatrixXd(7, 15);
  ukf.AugmentedSigmaPoints(&Xsig_aug);

  // print result
//   std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  return 0;
}