
#include <iostream>
#include "kinematics.h"
#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <iomanip>
const double M_PI = 3.14159265358979323846;



// We want to build the jacobian matrix for the robot. 
// This requires framing each Twist in the previous frame done with the Adjoint matrix
Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Matrix4d& T) {
    Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
    Ad.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
    Ad.block<3, 3>(3, 3) = T.block<3, 3>(0, 0);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d p = T.block<3, 1>(0, 3);
    Ad.block<3, 3>(0, 3) = skew(p) * R;
    return Ad;
}


//-------------------------------------------------------
// ----------Build Jacobian -----------------------------
//-------------------------------------------------------

// We calculate the jacobian matrix for the end effector (Spatial Velocity)
// The jacobian matrix is a 3xN matrix where N is the number of joints

Eigen::Matrix<double, 3, Eigen::Dynamic> jacobian(const std::vector<Twist>& twists, const Eigen::VectorXd& thetas, const Eigen::Vector3d& q_a) {
    Eigen::Matrix<double, 3, Eigen::Dynamic> Vs(3, twists.size());
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i = 0; i < twists.size(); ++i) {
        Eigen::Matrix<double, 6, 1> adjTwist = adjoint(T) * twistToVector(twists[i]);
        //std::cout << "Adjoint Twist of " << i << " is " << adjTwist << std::endl;
        Eigen::Matrix<double, 3, 1> omega = adjTwist.tail<3>();
        Eigen::Matrix<double, 3, 1> v = adjTwist.head<3>();
        Eigen::Matrix<double, 3, 1> linearVelocity = omega.cross(q_a) + v;

        Vs.col(i) = linearVelocity;

        T = T * calculateExpMap(twists[i], thetas(i));
    }
    return Vs;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> computePseudoInverse(Eigen::Matrix<double, 3, Eigen::Dynamic>& jacob)
{
    //std::cout << jacob.completeOrthogonalDecomposition().pseudoInverse();
    return jacob.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::Vector4d FWD_Kinematics(const std::vector<Twist>& twists, const std::vector<double> thetas, const Eigen::Vector4d& end_effector) {
    int n = twists.size();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < n; ++i) {
            T *= calculateExpMap(twists[i], thetas[i]);
        }
    return T*end_effector;
    } 

Eigen::VectorXd updateTheta(const Eigen::VectorXd& i_theta, const Eigen::Vector4d& e, const std::vector<Twist>& twists, const Eigen::Vector4d& end_effector, double alpha) {
    Eigen::Matrix<double, 3, Eigen::Dynamic> J = jacobian(twists, i_theta, end_effector.head(3));
    Eigen::Matrix<double, Eigen::Dynamic, 3> invJ = computePseudoInverse(J);
    Eigen::VectorXd i_1_theta = i_theta + alpha * (invJ * e.head(3));
    return i_1_theta;
}

// Function requires the twists, goal thetas, actual thetas, and the end effector position
// The function will return the change thetas until the end effector to the desired position
Eigen::VectorXd InverseKinematicsSolver(std::vector<Twist>& twists, Eigen::Vector4d& goal_pos, std::vector<Radian>& actual_thetas, Eigen::Vector4d& end_effector) {
    double alpha = 0.1; // sample rate
    Eigen::VectorXd i_theta(actual_thetas.size());
    for (size_t i = 0; i < actual_thetas.size(); ++i) {
        i_theta[i] = actual_thetas[i];
    }
    Eigen::Vector4d desired_pos = goal_pos;
    Eigen::Vector4d actual_pos = FWD_Kinematics(twists, std::vector<double>(i_theta.data(), i_theta.data() + i_theta.size()), end_effector);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Initial desired position is " << desired_pos.transpose() << std::endl; // in homogeneous coordinates (x, y, z, 1)
    std::cout << "Initial actual position is " << actual_pos.transpose() << std::endl; // in homogeneous coordinates (x, y, z, 1)
    Eigen::Vector4d e = desired_pos - actual_pos;
    //std::cout << "Initial error is " << e.transpose() << std::endl;

    while ((std::abs(e(0)) > 0.01) || (std::abs(e(1)) > 0.01) || (std::abs(e(2)) > 0.01)) {
        i_theta = updateTheta(i_theta, e, twists, end_effector, alpha);
        //std::cout << "Updated thetas are " << i_theta.transpose() << std::endl;
        actual_pos = FWD_Kinematics(twists, std::vector<double>(i_theta.data(), i_theta.data() + i_theta.size()), end_effector);
        std::cout << "Actual position is " << actual_pos.transpose() << std::endl;
        std::cout << "Desired position is " << desired_pos.transpose() << std::endl;
        e = desired_pos - actual_pos;
    }
    std::cout << "Actual position is " << actual_pos.transpose() << std::endl;
    std::cout << "Desired position is " << desired_pos.transpose() << std::endl;
    return i_theta;
}


int main()
{
    std::vector<double> lengths = {1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<Eigen::Vector3d> axes = {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1,0,0}, {0,1,0}};
    std::cout << "Enter the goal position in the format x y z" << std::endl;
    double x, y, z;
    std::cin >> x >> y >> z;
    Eigen::Vector4d goal_pos = Eigen::Vector4d(x, y, z, 1);
    std::vector<Radian> actual_thetas = {0, 0, 0, 0, 0};
    Eigen::Vector4d end_effector = Eigen::Vector4d(4, 0, 0, 1); // Explictly giving position of end-effector in homogeneous coordinates with thetas = 0
    double x_position = 0;
    std::vector<Twist> twists{};
    for (int i = 0; i < axes.size(); ++i) {
            Twist twist;
            twist.omega = axes[i]; 
            Eigen::Vector3d p;
            
            if (i == 0) {

                p = Eigen::Vector3d(0, 0, 0);
            } else {
                x_position = std::accumulate(lengths.begin(), lengths.begin() + i, 0.0);
                p = Eigen::Vector3d(x_position, 0, 0);
            }
            twist.v = -twist.omega.cross(p);
            
            twists.push_back(twist);   
            // std::cout << "Twist " << i << " is " << twist.v << " " << twist.omega << std::endl; // for debugging
        }
    
    InverseKinematicsSolver(twists, goal_pos, actual_thetas, end_effector);
    return 0;
}

