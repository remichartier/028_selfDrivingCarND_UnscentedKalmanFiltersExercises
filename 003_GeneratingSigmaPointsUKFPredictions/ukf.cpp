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
     • https://youtu.be/RQvnRpSPUak
    • This time your goal is to predict the Sigma points.
    • So we start from a point where we already have the augmented Sigma points.
    • Quickly repeating the meaning of each of these points.
        • 1st row px, py, v, Psi, Psi dot, noise longitudinal, noise yaw
    • Noise values are mostly 0s but not for all Sigma Points.
    • Make sure to also put these 2 noise values correctly into the process
       model.
    • Xsig_pred : Matrix you want to fill with the predicted Sigma Points.
    • And you need DeltaT if you want to calculate numbers.
    • Main thing you have to do here is implement the CTRV model into C++ code.
    • Make sure to also consider the effect of the process noise, and to catch 
      division by 0.

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

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; // time diff in sec

  /**
   * Student part begin
   */

  // predict sigma points
  // avoid division by zero
  // set xk vect state

  VectorXd xk = VectorXd(n_x);
  VectorXd fk = VectorXd(n_x);
  VectorXd mu_k = VectorXd(n_x);
  

  for(int i=0; i < (2*n_aug + 1);i++){
    xk = Xsig_aug.col(i).head(n_x);
    double px(xk(0)), py(xk(1)), v(xk(2)), psi(xk(3)),psiDot(xk(4));
    double mu_a(Xsig_aug.col(i)(5)), mu_psiDot(Xsig_aug.col(i)(6));
    if(psiDot!=0){
      fk(0) = (v/psiDot)*(sin(psi + psiDot*delta_t) - sin(psi));
      fk(1) = (v/psiDot)*(-cos(psi + psiDot*delta_t) + cos(psi));
      fk(3) = psiDot * delta_t;

    }
    else{
      fk(0) = v * cos(psi) * delta_t;
      fk(1) = v * sin(psi) * delta_t;;
      fk(3) = 0;
    }
    fk(2) = 0;
    fk(4) = 0;  

    mu_k(0) = 0.5*delta_t*delta_t*cos(psi)*mu_a;
    mu_k(1) = 0.5*delta_t*delta_t*sin(psi)*mu_a;
    mu_k(2) = delta_t*mu_a;
    mu_k(3) = 0.5*delta_t*delta_t*mu_psiDot;
    mu_k(4) = delta_t*mu_psiDot;

    // write predicted sigma points into right column

    Xsig_pred.col(i) = xk + fk + mu_k;
  }
  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  *Xsig_out = Xsig_pred;
}

/* Result obtained : 

Xsig_pred = 
 5.93553  6.06251  5.92217   5.9415  5.92361  5.93516  5.93705  5.93553  5.80832  5.94481  5.92935  5.94553  5.93589  5.93401  5.93553
 1.48939  1.44673  1.66484  1.49719    1.508  1.49001  1.49022  1.48939   1.5308  1.31287  1.48182  1.46967  1.48876  1.48855  1.48939
  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.23954   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049  2.17026   2.2049
 0.53678 0.473387 0.678098 0.554557 0.643644 0.543372  0.53678 0.538512 0.600173 0.395462 0.519003 0.429916 0.530188  0.53678 0.535048
  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528 0.387441 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528 0.318159
*/