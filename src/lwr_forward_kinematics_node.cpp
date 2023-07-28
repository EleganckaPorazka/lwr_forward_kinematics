//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#include <iostream>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include "lwr_forward_kinematics/lwr_forward_kinematics.hpp"
#include "rrlib_interfaces/msg/jacobian_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
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
    void TopicCallback(const sensor_msgs::msg::JointState & msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    rclcpp::Publisher<rrlib_interfaces::msg::JacobianStamped>::SharedPtr publisher_jacobian_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    
    LWRForwardKinematics forward_kinematics_;
};

LWRForwardKinematicsNode::LWRForwardKinematicsNode()
: Node("lwr_forward_kinematics")
{
    using std::placeholders::_1;
    
    RCLCPP_INFO(this->get_logger(), "Starting the node.");
    
    // tool pose in the last link's frame (initially set to coincide with the last link's frame)
    std::vector<double> tool_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    this->declare_parameter("tool", tool_value);
    
    // create a joint_states subscriber to get the joint positions
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&LWRForwardKinematicsNode::TopicCallback, this, _1));
    // create a publisher for the tool pose
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fwd_kin_pose", 10);
    // create a publisher for the Jacobian
    publisher_jacobian_ = this->create_publisher<rrlib_interfaces::msg::JacobianStamped>("fwd_kin_jacobian", 10);
    // create a parameter subscriber 
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    
    // set a callback for this node's parameter "tool"
    auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "Received an update to parameter \"%s\".", 
          p.get_name().c_str()
          );
          SetTool(p);
      };
    cb_handle_ = param_subscriber_->add_parameter_callback("tool", cb);
}

void LWRForwardKinematicsNode::SetTool(const rclcpp::Parameter & p)
{
    // get the tool pose in the last link's frame
    if (p.as_double_array().size() != 7)
    {
        RCLCPP_WARN(this->get_logger(),
            "The parameter \"%s\" requires 7 elements to describe its pose in the last link's frame. The tool change is not accepted.",
            p.get_name().c_str()
            );
    }
    else
    {
        // if the tool pose is legit, then transform it into the Eigen vector
        Eigen::VectorXd tool_vector = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(p.as_double_array().data(), 7);
        // set the tool pose in the forward kinematics solver
        forward_kinematics_.SetTool(tool_vector);
        RCLCPP_INFO(this->get_logger(), "The tool change has been accepted.");
    }
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
