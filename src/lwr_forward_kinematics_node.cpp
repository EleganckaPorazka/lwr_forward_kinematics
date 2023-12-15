//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#include <iostream>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include "lwr_forward_kinematics/lwr_forward_kinematics.hpp"
#include "rrlib_interfaces/msg/jacobian_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace lwr_fwd_kin
{
class LWRForwardKinematicsNode : public rclcpp::Node
{
public:
    LWRForwardKinematicsNode();
    void SetTool(const rclcpp::Parameter & p);
    
private:
    rcl_interfaces::msg::SetParametersResult ParametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    void TopicCallback(const sensor_msgs::msg::JointState & msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    rclcpp::Publisher<rrlib_interfaces::msg::JacobianStamped>::SharedPtr publisher_jacobian_;
    
    LWRForwardKinematics forward_kinematics_;
};

LWRForwardKinematicsNode::LWRForwardKinematicsNode()
: Node("lwr_forward_kinematics")
{
    using std::placeholders::_1;
    
    RCLCPP_INFO(this->get_logger(), "Starting the node.");
    
    // create a joint_states subscriber to get the joint positions
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&LWRForwardKinematicsNode::TopicCallback, this, _1));
    // create a publisher for the tool pose
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fwd_kin_pose", 10);
    // create a publisher for the Jacobian
    publisher_jacobian_ = this->create_publisher<rrlib_interfaces::msg::JacobianStamped>("fwd_kin_jacobian", 10);
    // declare parameters and create the callback for handling their changes
    // tool pose in the last link's frame (initially set to coincide with the last link's frame)
    std::vector<double> tool_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    this->declare_parameter("tool", tool_value);
    // create a parameter callback
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LWRForwardKinematicsNode::ParametersCallback, this, _1));
    
    // call explicitly the ParametersCallback to handle the parameter changes from the launcher
    std::vector<std::string> param_names = {"tool"};
    std::vector<rclcpp::Parameter> parameters = this->get_parameters(param_names);
    rcl_interfaces::msg::SetParametersResult set_param_result = LWRForwardKinematicsNode::ParametersCallback(parameters);
}

rcl_interfaces::msg::SetParametersResult LWRForwardKinematicsNode::ParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    RCLCPP_INFO(this->get_logger(), "Received a parameter update.");
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "";
    
    for (const auto &param : parameters)
    {
        if (param.get_name() == "tool")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
            {
                if (param.as_double_array().size() == 7)
                {
                    Eigen::VectorXd tool_vector = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(param.as_double_array().data(), 7);
                    if (tool_vector.tail(4).sum() < 1.00001 && tool_vector.tail(4).sum() > 0.99999) //TODO: obviously cannot compare with 1.0, but check whether the current approach is a good idea
                    {
                        forward_kinematics_.SetTool(tool_vector);
                        result.successful = true;
                        Eigen::VectorXd tool_pose = forward_kinematics_.GetTool();
                        RCLCPP_INFO(this->get_logger(), "Parameter \"%s\" updated to value: [%lf, %lf, %lf, %lf, %lf, %lf, %lf].",
                        param.get_name().c_str(), tool_pose(0), tool_pose(1), tool_pose(2), tool_pose(3), tool_pose(4), tool_pose(5), tool_pose(6));
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(),
                            "The parameter \"%s\" shall have the last four elements -- the quaternions -- sum to 1, but now it sums to: %lf.",
                            param.get_name().c_str(), tool_vector.tail(4).sum());
                        result.successful = false;
                        result.reason = "Quaternions shall sum to 1.";
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                        "The parameter \"%s\" requires 7 elements to describe its pose in the last link's frame, not %ld.",
                        param.get_name().c_str(), param.as_double_array().size());
                    result.successful = false;
                    result.reason = "The tool pose requires 7 elements.";
                }
            }
        }
    }
    
    return result;
}

void LWRForwardKinematicsNode::TopicCallback(const sensor_msgs::msg::JointState & msg)
{
    if (msg.position.size() < forward_kinematics_.GetDOF())
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(),
            *this->get_clock(),
            5000,
            "To solve the forward kinematics problem, there are %ld joint variables required, not %ld.",
            forward_kinematics_.GetDOF(),
            msg.position.size()
            );
    }
    else
    {
        // get the joint position vector "q" from the JointState type msg
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(msg.position.data(), forward_kinematics_.GetDOF());
        
        // compute the end effector pose and the Jacobian matrix of the manipulator
        Eigen::VectorXd pose = forward_kinematics_.ComputeToolPose(q);
        Eigen::MatrixXd jacobian = forward_kinematics_.ComputeJacobian(q);
        
        // publish the computed tool pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "kuka_lwr_base_link";
        pose_msg.pose.position.x = pose(0);
        pose_msg.pose.position.y = pose(1);
        pose_msg.pose.position.z = pose(2);
        pose_msg.pose.orientation.x = pose(3);
        pose_msg.pose.orientation.y = pose(4);
        pose_msg.pose.orientation.z = pose(5);
        pose_msg.pose.orientation.w = pose(6);
        publisher_pose_->publish(pose_msg);
        
        // publish the computed jacobian
        rrlib_interfaces::msg::JacobianStamped jacobian_msg;
        jacobian_msg.header.stamp = this->now();
        jacobian_msg.header.frame_id = "kuka_lwr_base_link";
        std::vector<double> jacobian_data(jacobian.data(), jacobian.data() + jacobian.size());
        jacobian_msg.jacobian.jacobian_data = jacobian_data;
        publisher_jacobian_->publish(jacobian_msg);
    }
}

} // namespace lwr_fwd_kin

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lwr_fwd_kin::LWRForwardKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
