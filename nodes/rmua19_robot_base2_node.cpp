/*******************************************************************************
 *  Copyright (c) 2021 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "rmoss_ign_base/chassis_gimbal_controller.hpp"
#include "rmoss_ign_base/simple_shooter_controller.hpp"

using namespace rmoss_ign_base;

void declare_parameter_pid(const rclcpp::Node::SharedPtr& nh,
        PidParam &pid_param,std::string prefix){
    nh->declare_parameter(prefix+"_p", pid_param.p);
    nh->declare_parameter(prefix+"_i", pid_param.i);
    nh->declare_parameter(prefix+"_d", pid_param.d);
}

void get_parameter_pid(const rclcpp::Node::SharedPtr& nh,
        PidParam &pid_param,std::string prefix){
    pid_param.p=nh->get_parameter(prefix+"_p").as_double();
    pid_param.i=nh->get_parameter(prefix+"_i").as_double();
    pid_param.d=nh->get_parameter(prefix+"_d").as_double();
}

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("robot_base");
    auto ign_node = std::make_shared<ignition::transport::Node>();
    // parameters
    std::string world_name,robot_name;
    ros_node->declare_parameter("world_name","default");
    world_name = ros_node->get_parameter("world_name").as_string();
    ros_node->declare_parameter("robot_name");
    robot_name = ros_node->get_parameter("robot_name").as_string();
    // topic string 
    std::string ros_gimbal_state_topic = "gimbal_state";
    std::string ign_chassis_cmd_topic = "/"+robot_name+"/cmd_vel";
    std::string ign_gimbal_pitch_cmd_topic = "/model/"+robot_name+"/joint/gimbal_pitch_joint/cmd_vel";
    std::string ign_gimbal_yaw_cmd_topic = "/model/"+robot_name+"/joint/gimbal_yaw_joint/cmd_vel";
    std::string ign_joint_state_topic = "/world/"+world_name+"/model/"+robot_name+"/joint_state";
    std::string ign_chassis_imu_topic = "/world/"+world_name+"/model/"+robot_name+"/link/chassis/sensor/chassis_imu/imu";
    std::string ign_gimbal_imu_topic = "/world/"+world_name+"/model/"+robot_name+"/link/gimbal_pitch/sensor/gimbal_imu/imu";
    // pid parameters
    PidParam picth_pid_param,yaw_pid_param,chassis_pid_param;
    picth_pid_param.p=10;
    yaw_pid_param.p=10;
    yaw_pid_param.i=0.1;
    yaw_pid_param.d=1;
    yaw_pid_param.cmdmin=-10;
    yaw_pid_param.cmdmax=10;
    chassis_pid_param.p=1;
    chassis_pid_param.i=0.1;
    declare_parameter_pid(ros_node,picth_pid_param,"pitch_pid");
    declare_parameter_pid(ros_node,yaw_pid_param,"yaw_pid");
    declare_parameter_pid(ros_node,chassis_pid_param,"chassis_pid");
    get_parameter_pid(ros_node,picth_pid_param,"pitch_pid");
    get_parameter_pid(ros_node,yaw_pid_param,"yaw_pid");
    get_parameter_pid(ros_node,chassis_pid_param,"chassis_pid");
    // create ign-gimbal-encoder
    auto ign_gimbal_encoder =  std::make_shared<IgnJointEncoder>(
        ign_node,ign_joint_state_topic);
    // create ign-gimbal-imu
    auto ign_gimbal_imu =  std::make_shared<IgnImu>(
        ign_node,ign_gimbal_imu_topic);
    // create ign-chassis-cmd-publisher
    auto ign_chassis_cmd_publisher =  std::make_shared<IgnChassisCmdPublisher>(
        ign_node,ign_chassis_cmd_topic);
    // create ign-gimbal-cmd-publisher
    auto ign_gimbal_cmd_publisher =  std::make_shared<IgnGimbalCmdPublisher>(
        ign_node,ign_gimbal_pitch_cmd_topic,ign_gimbal_yaw_cmd_topic);
    // create chassis-gimbal controller
    auto chassis_gimbal_controller =  std::make_shared<ChassisGimbalController>(
        ros_node,"chassis_cmd","gimbal_cmd",
        ign_chassis_cmd_publisher,ign_gimbal_cmd_publisher,
        ign_gimbal_encoder,ign_gimbal_imu
    );
    chassis_gimbal_controller->setPitchPid(picth_pid_param);
    chassis_gimbal_controller->setYawPid(yaw_pid_param);
    chassis_gimbal_controller->setChassisPid(chassis_pid_param);
    chassis_gimbal_controller->setControlMode(false);
    // create shooter controller
    std::string ign_shooter_cmd_topic = "/"+robot_name+"/small_shooter/shoot";
    auto shooter_controller = std::make_shared<SimpleShooterController>(
        ros_node,"shoot_cmd",ign_shooter_cmd_topic);
    // run node until it's exited
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),2,true);
    executor.add_node(ros_node);
    executor.spin();
    //rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
