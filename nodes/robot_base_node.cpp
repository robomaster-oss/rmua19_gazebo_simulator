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
#include <rclcpp/rclcpp.hpp>
#include "rmua19_ignition_simulator/robot_base.hpp"

using namespace rmua19;

int main(int argc, char * argv[])
{
  //creat ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("simulated_robot_base");
  // create a node
  auto task_node = std::make_shared<RobotBase>(node);
  // run node until it's exited
  rclcpp::spin(node);
  //clean up 
  rclcpp::shutdown();
  return 0;
}

