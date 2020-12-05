/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include "rmua19_ignition_simulator/robot_base.hpp"

using namespace std;
using namespace rmua19;

RobotBase::RobotBase(rclcpp::Node::SharedPtr& nh)
{
    // ROS  and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    // Init and get params
    nh_->declare_parameter("ign_topic_pitch_ctrl", "gimbal_pitch_control");
    nh_->declare_parameter("ign_topic_yaw_ctrl", "gimbal_yaw_control");
    nh_->declare_parameter("ign_topic_chassis_ctrl", "chassis_control");
    nh_->declare_parameter("ign_topic_joint_state", "joint_state");
    nh_->declare_parameter("ros_topic_gimbal_state", "robot_base/gimbal_state");
    nh_->declare_parameter("ros_topic_chassis_cmd", "robot_base/chassis_cmd");
    nh_->declare_parameter("ros_topic_gimbal_cmd", "robot_base/gimbal_cmd");
    nh_->declare_parameter("ros_topic_shoot_cmd", "robot_base/shoot_cmd");
    auto ign_topic_pitch_ctrl = nh_->get_parameter("ign_topic_pitch_ctrl").as_string();
    auto ign_topic_yaw_ctrl = nh_->get_parameter("ign_topic_yaw_ctrl").as_string();
    auto ign_topic_chassis_ctrl = nh_->get_parameter("ign_topic_chassis_ctrl").as_string();
    auto ign_topic_joint_state = nh_->get_parameter("ign_topic_joint_state").as_string();
    auto ros_topic_gimbal_state = nh_->get_parameter("ros_topic_gimbal_state").as_string();
    auto ros_topic_chassis_cmd = nh_->get_parameter("ros_topic_chassis_cmd").as_string();
    auto ros_topic_gimbal_cmd = nh_->get_parameter("ros_topic_gimbal_cmd").as_string();
    auto ros_topic_shoot_cmd = nh_->get_parameter("ros_topic_shoot_cmd").as_string();
    //create ros pub and sub
    ros_gimbal_state_pub_ = nh_->create_publisher<rmoss_interfaces::msg::Gimbal>(ros_topic_gimbal_state, 10);
    ros_chassis_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(ros_topic_chassis_cmd, 10, std::bind(&RobotBase::chassisCb, this, std::placeholders::_1));
    ros_gimbal_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(ros_topic_gimbal_cmd, 10, std::bind(&RobotBase::gimbalCb, this, std::placeholders::_1));
    ros_shoot_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::ShootCmd>(ros_topic_shoot_cmd, 10, std::bind(&RobotBase::shootCb, this, std::placeholders::_1));
    gimbal_state_timer_=nh_->create_wall_timer(20ms, std::bind(&RobotBase::gimbalStateTimerCb, this));
    //create ignition pub and sub
    ign_pitch_ctrl_pub_ = std::make_unique<ignition::transport::Node::Publisher>(ign_node_->Advertise<ignition::msgs::Double>(ign_topic_pitch_ctrl));
    ign_yaw_ctrl_pub_ = std::make_unique<ignition::transport::Node::Publisher>(ign_node_->Advertise<ignition::msgs::Double>(ign_topic_yaw_ctrl));
    ign_chassis_ctrl_pub_ = std::make_unique<ignition::transport::Node::Publisher>(ign_node_->Advertise<ignition::msgs::Twist>(ign_topic_chassis_ctrl));
    ign_node_->Subscribe(ign_topic_joint_state, &RobotBase::ignJointStateCb, this);
}

void RobotBase::gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
    ignition::msgs::Double ign_msg;
    ign_msg.set_data(msg->position.pitch);
    ign_pitch_ctrl_pub_->Publish(ign_msg);
    ign_msg.set_data(msg->position.yaw);
    ign_yaw_ctrl_pub_->Publish(ign_msg);
}
void RobotBase::shootCb(const rmoss_interfaces::msg::ShootCmd::SharedPtr msg)
{
    std::cout << "TODO: shootCb,shoot num:" << msg->projectile_num << std::endl;
}

void RobotBase::chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
    ignition::msgs::Twist ign_msg;
    ign_msg.mutable_linear()->set_x(msg->twist.linear.x);
    ign_msg.mutable_linear()->set_y(msg->twist.linear.y);
    ign_msg.mutable_angular()->set_z(msg->twist.angular.z);
    ign_chassis_ctrl_pub_->Publish(ign_msg);
}

void RobotBase::gimbalStateTimerCb()
{
    std::lock_guard<std::mutex> lock(gimbal_state_mut_);
    ros_gimbal_state_pub_->publish(gimbal_state_);
}

void RobotBase::ignJointStateCb(const ignition::msgs::Model& msg)
{
    std::lock_guard<std::mutex> lock(gimbal_state_mut_);
    gimbal_state_.pitch = msg.joint(0).axis1().position();
    gimbal_state_.yaw = msg.joint(1).axis1().position();
}