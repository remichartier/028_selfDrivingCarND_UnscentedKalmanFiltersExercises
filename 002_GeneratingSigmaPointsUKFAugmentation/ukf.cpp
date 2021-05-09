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
 *     •  https://youtu.be/5p-PqtxQeM8
    •     • Let’s put the augmentation into C++ code.
    • Now the dimension of the augmented state is 7.
    • Other things you will need are the variance of the longitudinal acceleration and the yaw acceleration.
    • Together, they build the process noise covariance matrix which we call Q.
    • The rest is almost the same as in the last session.
    • But this time you have to build the augmented state mean x_aug, and the augmented covariance matrix P_aug.
    • Look at cheat sheet, it will also help you here.
    • When you build the augmented mean state, consider that the mean value of the acceleration noises are both zero.
    • And this is where the augmented Sigma points go.
    • Pay attention to the dimensions of this matrix.

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

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  /**
   * Student part begin
   */
 
  // create augmented mean state

  // create augmented covariance matrix

  // create square root matrix

  // create augmented sigma points
  
  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  // write result
  *Xsig_out = Xsig_aug;
}

/** 
 * expected result:
 *  Xsig_aug =
 * 5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
 *   1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
 * 2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
 * 0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
 * 0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
 *      0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
 *      0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
 */