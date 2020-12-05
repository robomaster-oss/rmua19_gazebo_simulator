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
#ifndef RMUA19_IGNITION_SIMULATOR_ROBOT_BASE_H
#define RMUA19_IGNITION_SIMULATOR_ROBOT_BASE_H
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <ignition/transport/Node.hh>
#include "rmoss_interfaces/msg/chassis_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"


namespace rmua19
{

    class RobotBase
    {
    public:
        RobotBase(rclcpp::Node::SharedPtr &nh);
        ~RobotBase(){};
    private:
        void chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg);
        void gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);
        void shootCb(const rmoss_interfaces::msg::ShootCmd::SharedPtr msg);
        void gimbalStateTimerCb();
        //callback for ignition simulator
        void ignJointStateCb(const ignition::msgs::Model &msg);
    private:
        rclcpp::Node::SharedPtr nh_;
        std::shared_ptr<ignition::transport::Node> ign_node_;
        // ros pub and sub
        rclcpp::Publisher<rmoss_interfaces::msg::Gimbal>::SharedPtr ros_gimbal_state_pub_;
        rclcpp::Subscription<rmoss_interfaces::msg::ChassisCmd>::SharedPtr ros_chassis_cmd_sub_;
        rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr ros_gimbal_cmd_sub_;
        rclcpp::Subscription<rmoss_interfaces::msg::ShootCmd>::SharedPtr ros_shoot_cmd_sub_;
        rclcpp::TimerBase::SharedPtr gimbal_state_timer_;
        //ignition pub 
        std::unique_ptr<ignition::transport::Node::Publisher> ign_chassis_ctrl_pub_;
        std::unique_ptr<ignition::transport::Node::Publisher> ign_pitch_ctrl_pub_;
        std::unique_ptr<ignition::transport::Node::Publisher> ign_yaw_ctrl_pub_;
        // data
        rmoss_interfaces::msg::Gimbal gimbal_state_;
        std::mutex gimbal_state_mut_;
    };

} // namespace rmua19

#endif //RMUA19_IGNITION_SIMULATOR_ROBOT_BASE_H