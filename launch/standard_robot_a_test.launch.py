# Copyright 2021 RoboMaster-OSS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from xmacro.xmacro4sdf import XMLMacro4sdf

def generate_launch_description():
    ld = LaunchDescription()
    pkg_rmua19_gazebo_simulator = get_package_share_directory('rmua19_gazebo_simulator')
    world_sdf_path = os.path.join(pkg_rmua19_gazebo_simulator, 'resource', 'worlds', 'rmua19_world.sdf')
    robot_xmacro_path = os.path.join(pkg_rmua19_gazebo_simulator, 'resource', 'xmacro', 'rmua19_standard_robot_a.sdf.xmacro')
    ign_config_path = os.path.join(pkg_rmua19_gazebo_simulator, 'resource', 'ign', 'gui.config')
    robot_config = os.path.join(pkg_rmua19_gazebo_simulator, 'config', 'base_params.yaml')
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': world_sdf_path + ' -v 4 --gui-config ' + ign_config_path,
        }.items()
    )
    ld.add_action(gazebo)

    robot_names=["red_standard_robot1"]
    # Spawn robot
    robot_macro = XMLMacro4sdf()
    robot_macro.set_xml_file(robot_xmacro_path)
    robot_macro.generate({'global_initial_color': 'red'})
    robot_xml = robot_macro.to_string()
    spawn1 = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-name', robot_names[0] ,'-x', '-3.5','-y', '-2','-z', '0.08', '-string', robot_xml],
        output='screen')
    ld.add_action(spawn1)
    # robot base for each robot
    for robot_name in robot_names:
        robot_base = Node(
            package='rmoss_gz_base',
            executable='rmua19_robot_base',
            namespace=robot_name,
            parameters=[
                robot_config,
                {'robot_name': robot_name},
            ],
            output='screen')
        robot_ign_bridge = Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            namespace=robot_name,
            arguments=[
                '/world/default/model/%s/link/front_industrial_camera/sensor/front_industrial_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image'%(robot_name),
                '/world/default/model/%s/link/front_rplidar_a2/sensor/front_rplidar_a2/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'%(robot_name),
            ],
            remappings=[
                ('/world/default/model/%s/link/front_industrial_camera/sensor/front_industrial_camera/image'%(robot_name),'front_camera/image'),
                ('/world/default/model/%s/link/front_rplidar_a2/sensor/front_rplidar_a2/scan'%(robot_name),'rplidar_a2/scan'),
            ],
            output='screen'
        )
        ld.add_action(robot_base)
        ld.add_action(robot_ign_bridge)
    return ld
