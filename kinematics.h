#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <iostream>
#include <vector>
#include <../eigen-3.4.0/Eigen/dense>
#include <../eigen-3.4.0/unsupported/Eigen/MatrixFunctions>
using Radian = double;


struct Twist {
    Eigen::Vector3d v;
    Eigen::Vector3d omega;
};
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d mat;
    mat << 0, -v.z(), v.y(),
           v.z(), 0, -v.x(),
          -v.y(), v.x(), 0;
    return mat;
}

Eigen::Matrix<double, 6, 1> twistToVector(const Twist& twist) {
    Eigen::Matrix<double, 6, 1> vec;
    vec << twist.v, twist.omega;
    return vec;
}

Eigen::Matrix4d twistToHat(const Twist& twist) {
    Eigen::Matrix4d twistHat = Eigen::Matrix4d::Zero(); // Initialize a 4x4 matrix with zeros

    // Construct the skew-symmetric matrix for omega
    Eigen::Matrix3d omegaHat;
    omegaHat <<     0,             -twist.omega.z(),  twist.omega.y(),
                   twist.omega.z(),   0,             -twist.omega.x(),  
                  -twist.omega.y(),  twist.omega.x(),  0;

    // Set the top-left 3x3 block to omegaHat
    twistHat.block<3,3>(0, 0) = omegaHat;

    // Set the top-right 3x1 block to v
    twistHat.block<3,1>(0, 3) = twist.v;

    // The bottom row remains [0, 0, 0, 0]
    return twistHat;
}
Eigen::Matrix4d calculateExpMap(const Twist& twist, Radian theta) {
    // Scale the twist by theta before converting to "hat" form
    Twist scaledTwist;
    scaledTwist.v = twist.v * theta;
    scaledTwist.omega = twist.omega * theta;

    // Now, convert the scaled twist to "hat" form and compute the exponential
    Eigen::Matrix4d m = twistToHat(scaledTwist);
    return m.exp();
}

#endif
