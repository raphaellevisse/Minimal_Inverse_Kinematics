# Minimal_Inverse_Kinematics
Straight-forward use Inverse Kinematics library, taking axis, lengths and a goal position and solves the inverse kinematics using Jacobian Gradient Descent.

Stemmed from prefering Matrix Exponential for robotic transformations and a more elegant jacobian calculation. 

The code here is very general and not optimized (change Eigen::VectorXd and Eigen::Dynamic to your robot to gain speed). This program will run for revolute joints of any kind and can be simply adapted to prismatic joints. 


## Key Features

- **Simplicity:** Just specify axes, lengths, and a goal position.
- **Flexibility:** Compatible with revolute joints of any kind and easily adaptable for prismatic joints (simply adapt twist calculation)
- **Matrix Exponential:** Inspired by a preference for Matrix Exponential methods for robotic transformations and a refined approach to Jacobian calculation, offering a more elegant and general solution than Denavit-Hartenberg parameters.
- **General and Adaptable:** The provided code is intentionally general, allowing you to customize and optimize it for your specific robotic setup to achieve enhanced performance.

## Getting Started

1. **Set Up Your Environment:** Ensure that you have Eigen installed, as this library relies on Eigen for matrix and vector operations.
2. **Integrate with Your Project:** Import the Minimal Inverse Kinematics library into your project.
3. **Configure Your Robot Parameters:** Customize the library's settings, such as `Eigen::VectorXd` and `Eigen::Dynamic`, to match your robot's specifications for improved efficiency.
4. **Define Your Goal:** Change the axis, lengths, and goal position for your robotic arm.
5. **Solve:** Execute the solver to obtain the inverse kinematics solution for your robot.

I put a pre-existing robot as a starter code.
