# Minimal_Inverse_Kinematics
Extremely simple to use Inverse Kinematics library, taking axis, lengths and a goal position and solves the inverse kinematics using Jacobian Gradient Descent.

Stemmed from prefering Matrix Exponential for robotic transformations and a more elegant jacobian calculation. 

The code here is very general and not optimized (change Eigen::VectorXd and Eigen::Dynamic to your robot to gain speed). This program will run for revolute joints of any kind and can be simply adapted to prismatic joints. 

