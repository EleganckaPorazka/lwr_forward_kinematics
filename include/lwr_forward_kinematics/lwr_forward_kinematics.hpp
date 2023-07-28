//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#ifndef LWR_FWD_KIN_H
#define LWR_FWD_KIN_H

#include <eigen3/Eigen/Dense>
#include "helper_functions.hpp"

namespace lwr_fwd_kin
{
class LWRForwardKinematics
{
public:
    LWRForwardKinematics();
    void SetTool( const Eigen::VectorXd &tool_pose );
    size_t GetDOF();
    Eigen::VectorXd ComputeToolPose( const Eigen::VectorXd &q );
    Eigen::MatrixXd ComputeJacobian( const Eigen::VectorXd &q );
    
private:
    size_t DOF_;    // number of degrees of freedom of the manipulator
    // modified Denavit-Hartenberg parameters:
    Eigen::VectorXd a_;   // rotation angles around "x_{j - 1}" axis
    Eigen::VectorXd d_;   // translational displacements along "z_j" axis

    Eigen::Matrix4d A_tool_;  // transformation matrix from the last link frame to the tool frame
};

LWRForwardKinematics::LWRForwardKinematics()
: DOF_(7)
{
    a_.resize(DOF_);
    d_.resize(DOF_);
    // Important: pay attention whether the values of a_ and d_ correspond to the xacro file!
    a_ << 0.0, M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2;
    d_ << 0.3105, 0.0, 0.4, 0.0, 0.39, 0.0, 0.078;
    
    A_tool_ = Eigen::Matrix4d::Identity();
}

void LWRForwardKinematics::SetTool( const Eigen::VectorXd &tool_pose )
{
    Eigen::Vector3d r_tool = tool_pose.head(3);
    Eigen::Vector4d Q_tool = tool_pose.tail(4);
    A_tool_.block<3, 3>(0, 0) = rrlib::QuaternionsToRotationMatrix( Q_tool );
    A_tool_.block<3, 1>(0, 3) = r_tool;
    A_tool_.row(3) << 0.0, 0.0, 0.0, 1.0;
}

size_t LWRForwardKinematics::GetDOF()
{
    return DOF_;
}

Eigen::VectorXd LWRForwardKinematics::ComputeToolPose( const Eigen::VectorXd &q )
{
    Eigen::Matrix4d A_i, A_ij, A_j; // i = j - 1
    A_i = Eigen::Matrix4d::Identity();
    
    for (size_t j = 0; j < DOF_; j++)
    {
        A_ij << cos( q(j) ),                -sin( q(j) ),               0.0,            0.0,
                sin( q(j) ) * cos( a_(j) ), cos( q(j) ) * cos( a_(j) ), -sin( a_(j) ),  -d_(j) * sin( a_(j) ),
                sin( q(j) ) * sin( a_(j) ), cos( q(j) ) * sin( a_(j) ), cos( a_(j) ),   d_(j) * cos( a_(j) ),
                0.0,                        0.0,                        0.0,            1.0;
        
        A_j = A_i * A_ij;
        A_i = A_j;
    }
    A_j *= A_tool_;
    
    Eigen::Vector3d position = A_j.block<3, 1>(0, 3);
    Eigen::Matrix3d R = A_j.block<3, 3>(0, 0);
    Eigen::Vector4d orientation = rrlib::RotationMatrixToQuaternions( R );
    
    Eigen::VectorXd pose(7);
    pose.head(3) = position;
    pose.tail(4) = orientation;
    return pose;
}

Eigen::MatrixXd LWRForwardKinematics::ComputeJacobian( const Eigen::VectorXd &q )
{
    Eigen::Matrix4d A_i, A_ij, A_j; // i = j - 1
    A_i = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd Z(3, DOF_);  // each column is the axis of rotation of the j-th joint (in our case: "z" axis od the j-th link frame), expressed in the base frame
    Eigen::MatrixXd P(3, DOF_);  // each column is the vector from the base to the origin of the j-th link frame, expressed in the base frame
    
    for (size_t j = 0; j < DOF_; j++)
    {
        A_ij << cos( q(j) ),                -sin( q(j) ),               0.0,            0.0,
                sin( q(j) ) * cos( a_(j) ), cos( q(j) ) * cos( a_(j) ), -sin( a_(j) ),  -d_(j) * sin( a_(j) ),
                sin( q(j) ) * sin( a_(j) ), cos( q(j) ) * sin( a_(j) ), cos( a_(j) ),   d_(j) * cos( a_(j) ),
                0.0,                        0.0,                        0.0,            1.0;
        
        A_j = A_i * A_ij;
        P.col(j) = A_j.block<3, 1>(0, 3);
        Z.col(j) = A_j.block<3, 1>(0, 2);
        A_i = A_j;
    }
    A_j *= A_tool_;
    
    Eigen::Vector3d p_tool = A_j.block<3,1>(0,3);   // vector from the base frame to the tool frame, expressed in the base frame
    
    Eigen::MatrixXd J(6, DOF_);  // jacobian
    for (size_t j = 0; j < DOF_; j++)
	{
		J.col(j).head(3) = Eigen::Vector3d(Z.col(j)).cross( p_tool - P.col(j) );
        J.col(j).tail(3) = Z.col(j);
	}
    
    return J;
}

} // namespace lwr_fwd_kin

#endif // LWR_FWD_KIN_H
