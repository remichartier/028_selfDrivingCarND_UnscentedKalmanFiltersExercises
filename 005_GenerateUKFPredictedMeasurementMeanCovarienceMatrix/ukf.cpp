#include <iostream>
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}

/**
       • https://youtu.be/GYQeizoj09E
    • Goal coding Quiz is to predict the mean and the covariance of a radar
     measurement.
    • Expect Radar measurement → measurement dimension to 3.
    • Gave example on how set the weights.
    • To be able to build the measurement covariance matrix R, gave values
     for radar measurement uncertainty.
    • Predicted Sigma Points given.
    • Zsig to store measurement Sigma points.
    • 2 output objects : z_pred mean predicted measurement, and S the predicted
     covariance matrix S.

 */

// compilation : g++ main.cpp ukf.cpp -o main.exe

/* Dense not found --> due to Eigen files/lib not installed 
  From : https://eigen.tuxfamily.org/dox/GettingStarted.html
   Compiling and running your first program
  There is no library to link to. The only thing that you need to keep in mind when compiling the above program is that the compiler must be able to find the Eigen header files. The directory in which you placed Eigen's source code must be in the include path. With GCC you use the -I option to achieve this, so you can compile the program with a command like this:

  g++ -I /path/to/eigen/ my_program.cpp -o my_program 
  On Linux or Mac OS X, another option is to symlink or copy the Eigen folder into /usr/local/include/. This way, you can compile the program with:

  g++ my_program.cpp -o my_program 
  
  https://askubuntu.com/questions/860207/how-to-install-eigen-3-3-in-ubuntu-14-04
  For those simply requiring a reasonably recent version of Eigen 3 on Ubuntu
   and similar Debian-based distros (...which is the common case), installing 
   the existing libeigen3-dev package suffices: e.g.,

  sudo apt install libeigen3-dev

*/

/**
 * Programming assignment functions: 
 */

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  double weight = 0.5/(lambda+n_aug);
  weights(0) = weight_0;

  for (int i=1; i<2*n_aug+1; ++i) {  
    weights(i) = weight;
  }

  // radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  // radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  // radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  // create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  /**
   * Student part begin
   */

  // transform sigma points into measurement space
  VectorXd x = VectorXd(n_x);
  VectorXd z = VectorXd(n_z);
  
  for(int i=0; i<2 * n_aug + 1; i++){
	x = Xsig_pred.col(i);
	// Column i
	double px(x(0)), py(x(1)), v(x(2)), psi(x(3)), psi_dot(x(4));
	double rho = sqrt(px*px + py*py);
	if(px==0){
		std::cout << "px=0 division by 0" << std::endl;
	  	return;
	}
	double phi = atan2(py,px);
	if(rho==0){
	  	std::cout << "sqrt(px*px + py*py)=0 division by 0" << std::endl;
	  	return;
	}
	double phi_dot = v*(px*cos(psi) + py*sin(psi))/rho;
	z(0) = rho;
	z(1) = phi;
	z(2) = phi_dot;
	Zsig.col(i) = z;  	
  }
  
  // calculate mean predicted measurement
  z_pred = weights(0)*Zsig.col(0);
  for(int i=1; i<2 * n_aug + 1; i++){
  	z_pred = z_pred + weights(i)*Zsig.col(i);
  }
  
  // calculate innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<
  		std_radr*std_radr, 	0, 						0,
  		0, 					std_radphi*std_radphi,	0,
  		0,					0,						std_radrd*std_radrd;

  S.Zero(n_z,n_z);
  VectorXd diff = VectorXd(n_z);
  for(int i=0; i<2 * n_aug + 1; i++){
  	diff = (Zsig.col(i) - z_pred);
  	S = S + (weights(i)*diff*(diff.transpose()));
  }
  S = S + R;
  /**
   * Student part end
   */

  // print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  // write result
  *z_out = z_pred;
  *S_out = S;
}