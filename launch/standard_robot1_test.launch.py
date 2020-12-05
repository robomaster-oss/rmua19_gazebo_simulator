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

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_rmua19_ignition_simulator = get_package_share_directory(
        'rmua19_ignition_simulator')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    # Spawn dolly
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                     '-name', 'standard_robot',
                     '-x', '5.0',
                     '-z', '0.46',
                     '-Y', '1.57',
                     '-file', os.path.join(pkg_rmua19_ignition_simulator, 'models', 'rmua19_standard_robot1',
                                           'model.sdf')],
                 output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ign_args',
            default_value=[os.path.join(pkg_rmua19_ignition_simulator, 'worlds', 'empty_world.sdf') 
                            + ' -v 2 --gui-config ' +
                            os.path.join(pkg_rmua19_ignition_simulator, 'ign', 'gui.config'), ''],
            description='Ignition Gazebo arguments'),
        gazebo,
        spawn
    ])
