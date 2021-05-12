# 028_selfDrivingCarND_UnscentedKalmanFiltersExercises
Quiz/Exercises done during Udacity Self-Driving Car Nanodegree extra-curriculum program


- 001_GeneratingSigmaPointsAssignment01 : write a small function that generates Sigma Points. Using Eigen C++ lib/header for matrices. Using the complete state of the CTRV model (5 dimensions). Need to calculate all the 11 Sigma Points and fill columns of Xsig.

- 002_GeneratingSigmaPointsUKFAugmentation : Adapt UKF Sigma point generation function to the augmented state. Build the augmented state mean x_aug, and the augmented covariance matrix P_aug. Generate Augmented Sigma Points Matrix.

- 003_GeneratingSigmaPointsUKFPredictions : Predict the UKF Sigma points for the CTRV model.

- 004_GenerateUKFPredictedMeanAndCovarianceMatrix : 2 objects we want to calculate : the mean predicted state X and the state prediction covariance P.

- 005_GenerateUKFPredictedMeasurementMeanCovarienceMatrix : Coding Quiz to predict the mean and the covariance of a radar measurement. 2 output objects : z_pred mean predicted measurement, and S the predicted covariance matrix S.

