#include "ukf.h"
#include <iostream>

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
 *     •  https://youtu.be/TIc3n-cxTqc
    • First Coding quiz in this session.
    • Let’s go ahead and put what we have learned into C++.
    • You will write a small function that generates Sigma Points.
    • Template prepared for you.
    • Work on Function called GenerateSigmaPoints()  receives a pointer to a Matrix as input. This is where you are expected to write your result.
    • This time we will consider the complete state of the CTRV model.
    • So we set the state dimension nx to 5.
    • Lamda = 3 – nx as suggested before.
    • We set example set x and example Covariance matrix P to some realistic values.
    • Matrix Xsig, 5 rows, 11 columns, to store the Sigma Points later.
    • Calculation of square root of P, 2 functions part of the Eigen Library. Performs a Cholesky decomposition and provides the result we need.
    • Need to calculate all the 11 Sigma Points and fill columns of Xsig.
    • One important thing : when you fill this matrix, use the same ordering of Sigma points as showed in the video, to make sure the evaluation works.
    • Result printed at the end, so you can check if it’s realistic.
    • Some of the challenges of these programming assignments are related to handling rows and columns of matrices with the Eigen library.
    • Check the link to the Eigen Cheat Sheet in the assignment description, that will help you a lot.
Instructor Notes:
In the video, the upper left value of sqrt(Pk​∣k) is 0.00656. It should actually be 0.0656

Generating Sigma Points Assignment
You will be completing the missing code in ukf.cpp, GenerateSigmaPoints() function.
Task List
    • Calculate the sigma points and store them in the input reference matrix 
 */

// compilation : g++ main.cpp ukf.cpp -o main

/* Dense not found --> due to Eigen files/lib not installed in miniconda
   https://anaconda.org/conda-forge/eigen
   --> conda install -c conda-forge eigen

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
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // define spreading parameter
  double lambda = 3 - n_x;

  // set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // set example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  // calculate square root of P
  MatrixXd A = P.llt().matrixL();

  /**
   * Student part begin
   */

  // your code goes here 
  // calculate sigma points ...
  Xsig.col(0) = x;

  // set sigma points as columns of matrix Xsig
  for(int i=0; i < n_x; i++){
    //std::cout << "i = " << i << std::endl;
    Xsig.col(i+1)     = x + sqrt(lambda + n_x) * A.col(i);
    Xsig.col(i+1+n_x) = x - sqrt(lambda + n_x) * A.col(i);
  }
  /**
   * Student part end
   */

  // print result
  // std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  // write result
  *Xsig_out = Xsig;
}

/**
 * expected result:
 * Xsig =
 *  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441
 *    1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38
 *  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049
 *  0.5015  0.44339 0.631886 0.516923 0.595227   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015
 *  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879
 */

/* Results obtained : 
Xsig = 
  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441
    1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38
  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049
  0.5015  0.44339 0.631886 0.516923 0.595227   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015
  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879


*/