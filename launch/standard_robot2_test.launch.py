#  Copyright (c) 2020 robomaster-oss, All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the MIT License, See the MIT License for more details.
#
#  You should have received a copy of the MIT License along with this program.
#  If not, see <https://opensource.org/licenses/MIT/>.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "standard_robot_red1"
    pkg_rmua19_ignition_simulator = get_package_share_directory('rmua19_ignition_simulator')
    world_sdf_path = os.path.join(pkg_rmua19_ignition_simulator, 'worlds', 'rmua19_world.sdf')
    robot_r1_sdf_path = os.path.join(pkg_rmua19_ignition_simulator, 'models', 'rmua19_standard_robot2_red1','model.sdf')
    robot_b2_sdf_path = os.path.join(pkg_rmua19_ignition_simulator, 'models', 'rmua19_standard_robot2_blue2','model.sdf')
    ign_config_path = os.path.join(pkg_rmua19_ignition_simulator, 'ign', 'gui.config')
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': world_sdf_path + ' -v 4 --gui-config ' + ign_config_path,
        }.items()
    )
    # Spawn robot
    spawn1 = Node(package='ros_ign_gazebo', executable='create',
        arguments=['-name', robot_name,'-x', '-1','-y', '-0.5','-z', '0.1',
                    '-file', robot_r1_sdf_path],
        output='screen')
    spawn2 = Node(package='ros_ign_gazebo', executable='create',
        arguments=['-name', "standard_robot_blue1",'-x', '2','-y', '-0.2','-z', '0.1',
                    '-file', robot_b2_sdf_path],
        output='screen')
    # robot base
    chassis_controller =Node(package='rmoss_ign_robot_base', executable='chassis_simple_controller',
        namespace= robot_name+"/robot_base",
        parameters=[
            {'ign_chassis_cmd_topic': "/%s/cmd_vel"%(robot_name)},
        ],
        output='screen') 
    gimbal_controller =Node(package='rmoss_ign_robot_base', executable='gimbal_simple_controller',
        namespace= robot_name+"/robot_base",
        parameters=[
            {'ign_pitch_topic': "/model/%s/joint/gimbal_pitch_joint/0/cmd_pos"%(robot_name)},
            {'ign_yaw_topic': "/model/%s/joint/gimbal_yaw_joint/0/cmd_pos"%(robot_name)},
        ],
        output='screen') 
    gimbal_publisher =Node(package='rmoss_ign_robot_base', executable='gimbal_state_publisher',
        namespace= robot_name+"/robot_base",
        parameters=[
            {'ign_topic': "/world/demo/model/%s/joint_state"%(robot_name)},
        ],
        output='screen') 
    shooter_controller =Node(package='rmoss_ign_robot_base', executable='shooter_simple_controller',
        namespace= robot_name+"/robot_base",
        parameters=[
            {'ign_shoot_cmd_topic': "/%s/small_shooter/shoot"%(robot_name)},
        ],
        output='screen') 
    # Bridge
    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=["/world/demo/model/%s/link/front_industrial_camera/sensor/front_industrial_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image"%(robot_name),
                   "/world/demo/model/%s/link/chassis/sensor/chassis_imu/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"%(robot_name),
                   "/world/demo/model/%s/link/front_rplidar_a2/sensor/front_rplidar_a2/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"%(robot_name),
                   "%s/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"%(robot_name)
        ],
        remappings=[
            ("/world/demo/model/%s/link/front_industrial_camera/sensor/front_industrial_camera/image"%(robot_name),"%s/front_camera/image"%(robot_name)),
            ("/world/demo/model/%s/link/chassis/sensor/chassis_imu/imu"%(robot_name),"%s/chassis_imu"%(robot_name)),
            ("/world/demo/model/%s/link/front_rplidar_a2/sensor/front_rplidar_a2/scan"%(robot_name),"%s/laser_scan"%(robot_name)),
        ],
        output='screen'
    )
    return LaunchDescription([
        gazebo,
        spawn1,spawn2,
        chassis_controller,gimbal_controller,gimbal_publisher,shooter_controller,
        ign_bridge
    ])
